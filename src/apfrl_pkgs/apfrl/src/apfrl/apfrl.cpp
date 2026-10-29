#include <cmath>
#include <stdexcept>
#include <functional>   // std::plus
#include <algorithm>    // std::transform
#include <iostream>
#include "torch/cuda.h"
#include "apfrl/apfrl.h"

ObsProfile ParseObsProfile(const std::string &name) {
  if (name == "barn")   return ObsProfile::kBarn;
  if (name == "legacy") return ObsProfile::kLegacy;
  throw std::invalid_argument(
      "obs_profile must be \"barn\" or \"legacy\", got \"" + name + "\"");
}

double ApfRl::MaxRangeFor(ObsProfile profile) {
  return profile == ObsProfile::kLegacy ? kLegacyMaxRange : kBarnMaxRange;
}

ApfRl::ApfRl(std::string &model_path, ObsProfile profile)
    :is_initial_(true), prev_nav_state_(NavState::WAIT), profile_(profile){
  std::cout << "ApfRl observation profile: "
            << (profile_ == ObsProfile::kLegacy ? "legacy (clamp 150, /75)"
                                                : "barn (clamp 28, affine, goal /30)")
            << std::endl;

  std::cout << "Availability of torch::cuda " << torch::cuda::is_available() << std::endl;

  metrics_calculator_ = std::make_unique<MetricsCalculator>();
  prev_state_ = {};
  module_ = torch::jit::load(model_path); //Loads pytorch model, set path correctly!
  module_.to(at::kCUDA); //converts it to Cuda(Gpu) model
  // Warm-up forward pass (CUDA kernel compile / weight load); no autograd needed.
  torch::NoGradGuard no_grad;
  std::vector<torch::jit::IValue> inputs;
  torch::Tensor stateTensor = torch::zeros({1, STATE_SIZE}).to(at::kCUDA);
  inputs.push_back(stateTensor);
  (void)module_.forward(inputs).toTensor();
}
void ApfRl::Step(SocialInput &input, Output &output) {
  CheckStatus(input, output.status);
  
  if (output.status == NavState::RESUME){
    auto reset = (prev_nav_state_ != NavState::RESUME && prev_nav_state_ != NavState::ALMOST);
    metrics_calculator_->Update(input,reset);
    std::vector<float> state {};
    state.reserve(STATE_SIZE);
    GenerateState(input,state);
    GetAction(state,output);
  } else if (output.status == NavState::ALMOST){
    //std::cout << "ALMOST" << std::endl;
    output.wp_x = input.target.x;
    output.wp_y = input.target.y;
    output.apf_k = kMaxApfK;
  } else {
    //std::cout << "ELSE" << std::endl;
    output.wp_x = 0.0;
    output.wp_y = 0.0;
    output.apf_k = 1.0;
  }
  prev_nav_state_ = output.status;
  
  //std::cout << "APFRL - Step Ends" << std::endl;
}

void ApfRl::GenerateState(SocialInput &input, std::vector<float> &state) {

 // std::for_each(input.scan.begin(), input.scan.end(), [](double& d) { d-=kRhoRobot;});
  for (auto &range: input.scan) {
    range -= kRhoRobot;
  }

  if (is_initial_) {
    state.insert(state.end(),input.scan.begin(), input.scan.end());
    state.push_back(input.target.x);
    state.push_back(input.target.y);
    is_initial_ = false;
  } else {
    auto yaw_diff = input.robot_pose.yaw - prev_yaw_;
    // Wrap to [-pi, pi] before turning it into a bin shift. odom yaw is
    // atan2-bounded, so a crossing of +/-pi reads as a jump of nearly 2pi for a
    // rotation of a few degrees: going from +3.0 to -3.0 rad is -0.28 rad of
    // real motion but computes as -6.0, shifting the retained scan by 3 bins
    // where 62 is correct. Nothing crashes - the shift stays in range either way
    // - the policy is simply handed a scan history rotated to the wrong heading.
    yaw_diff = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));
    prev_yaw_ = input.robot_pose.yaw;
    int shift = static_cast<int>((STATE_SCAN_SIZE*yaw_diff)/(2.0*M_PI));
    // Rotate the same way the trainer does. APFRL_env_Ata.py applies
    // np.roll(old_scan, shift), which moves each element towards HIGHER indices;
    // std::rotate(first, first+n, last) moves them towards LOWER ones. Feeding
    // the raw shift to std::rotate turns the history by +shift where the trainer
    // turned it by -shift, so the retained scan is misaligned by twice the
    // rotation the robot just made. Rotating left by (N - shift) is np.roll.
    shift = ((shift % STATE_SCAN_SIZE) + STATE_SCAN_SIZE) % STATE_SCAN_SIZE;
    const int rot = (STATE_SCAN_SIZE - shift) % STATE_SCAN_SIZE;
    std::array<float,STATE_SCAN_SIZE> old_scan;
    std::copy(prev_state_.begin(),prev_state_.begin()+STATE_SCAN_SIZE,old_scan.begin());

    (void)std::rotate(old_scan.begin(), old_scan.begin()+rot, old_scan.end());
    state.insert(state.end(),old_scan.begin(), old_scan.end());
    state.push_back(prev_state_[STATE_SCAN_SIZE]);
    state.push_back(prev_state_[STATE_SCAN_SIZE+1]);
  }

  state.insert(state.end(),input.scan.begin(), input.scan.end());
  state.push_back(input.target.x);
  state.push_back(input.target.y);
  prev_state_ = {};
  std::copy(state.begin()+(STATE_SCAN_SIZE+2),state.end(),prev_state_.begin());

  // Normalise per channel and clip, exactly as the trainer does. Layout is
  //   [old_scan(N) | old_goal(2) | scan(N) | goal(2)]
  // with the scan mapped affinely onto [-1, 1] and the goal divided by
  // kGoalNorm. One shared constant cannot serve both.
  auto clip = [](float v) { return std::max(-1.0f, std::min(1.0f, v)); };
  if (profile_ == ObsProfile::kLegacy) {
    // Pre-BARN contract, verbatim: one divisor for every element, then a
    // per-component clip. Deliberately not the affine map -- apfrl_model.pt and
    // stackelberg_model.pt were fitted on exactly this, and "improving" it here
    // would be the train/deploy mismatch, not the fix for one.
    for (auto &val : state) {
      val /= static_cast<float>(kLegacyNorm);
      val = clip(val);
    }
    return;
  }
  // The goal is scaled by its magnitude, never per component -- clipping gx and
  // gy independently would bend the bearing, turning a goal at (40, 10) m into
  // 14 degrees instead of 18.3.
  auto norm_goal = [&](float &gx, float &gy) {
    gx /= static_cast<float>(kGoalNorm);
    gy /= static_cast<float>(kGoalNorm);
    const float m = std::hypot(gx, gy);
    if (m > 1.0f) { gx /= m; gy /= m; }
  };
  // Affine, not a division: a range is non-negative, so dividing alone could
  // never produce a value in [-1, 0) and half the space would go unused.
  auto norm_scan = [&](float v) {
    return clip(2.0f * v / static_cast<float>(kScanNorm) - 1.0f);
  };
  for (int i = 0; i < STATE_SCAN_SIZE; ++i) {
    state[i] = norm_scan(state[i]);
    state[STATE_SCAN_SIZE + 2 + i] = norm_scan(state[STATE_SCAN_SIZE + 2 + i]);
  }
  norm_goal(state[STATE_SCAN_SIZE], state[STATE_SCAN_SIZE + 1]);
  norm_goal(state[2 * STATE_SCAN_SIZE + 2], state[2 * STATE_SCAN_SIZE + 3]);
  state[STATE_SCAN_SIZE] = clip(state[STATE_SCAN_SIZE]);
  state[STATE_SCAN_SIZE + 1] = clip(state[STATE_SCAN_SIZE + 1]);
  state[2 * STATE_SCAN_SIZE + 2] = clip(state[2 * STATE_SCAN_SIZE + 2]);
  state[2 * STATE_SCAN_SIZE + 3] = clip(state[2 * STATE_SCAN_SIZE + 3]);
}

void ApfRl::GetAction(std::vector<float> &state, Output &output) {
  // NoGradGuard: inference-only path, no autograd tape -> no per-call leak.
  torch::NoGradGuard no_grad;
  std::vector<torch::jit::IValue> inputs;
  torch::Tensor stateTensor = torch::from_blob(state.data(),{1, STATE_SIZE}).to(at::kCUDA);
  inputs.push_back(stateTensor);
  at::Tensor actionT = module_.forward(inputs).toTensor().to(torch::kCPU); //get action using model
  std::vector<float> action(actionT.data_ptr<float>(), actionT.data_ptr<float>() + actionT.numel());
  output.wp_x = (action[0]+1)*kXScale/2;
  output.wp_y = action[1]*kYScale;
  output.apf_k = ((action[2]+1.5))*2;
}

void ApfRl::CheckStatus(const SocialInput &input, NavState &status) {
    bool collision = false; //*std::min_element(input.scan.begin(), input.scan.end()) < kCollisionDistance;
  auto goal_distance = std::pow(std::pow(input.target.x,2) + std::pow(input.target.y,2),0.5);
  bool reached =  goal_distance < kReachDistance;
  bool almost =  goal_distance < kCloseDistance;
  bool wait = (input.target.x == 0.0 && input.target.y == 0.0);

  if (wait) {
    status = NavState::WAIT;
  } else if(reached) {
    status = NavState::SUCCESS;
    std::cout << "SUCCESS!!"<<std::endl;
    metrics_calculator_->PrintMetrics();
  } else if(almost) {
    status = NavState::ALMOST;
  } else if(collision) {
    status = NavState::COLLISION;
    //std::cout << "COLLISION!!"<<std::endl;
    metrics_calculator_->PrintMetrics();
  } else {
    status = NavState::RESUME;
  }
}

double ApfRl::subtract_radius(double x) {  return (x-kRhoRobot); }
double ApfRl::normalize(double x) {  return (2.0*x/kScanNorm - 1.0); }

