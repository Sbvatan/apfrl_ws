
#include <cmath>
#include <algorithm>  // std::transform
#include <functional> // std::plus
#include <iostream>
#include "social_apfrl/social_apfrl.h"
SocialApfRl::SocialApfRl(std::string &model_path)
    : is_initial_(true), prev_nav_state_(NavState::WAIT) {
  metrics_calculator_ = std::make_unique<MetricsCalculator>();
  prev_state_ = {};
  module_ =
      torch::jit::load(model_path); // Loads pytorch model, set path correctly!
  module_.to(at::kCUDA);             // converts it to Cuda(Gpu) model
  std::vector<torch::jit::IValue> inputs;
  torch::Tensor stateTensor = torch::zeros({1, SOCIAL_STATE_SIZE}).to(at::kCUDA);
  inputs.push_back(stateTensor);
  (void)module_.forward(inputs).toTensor();
}
void SocialApfRl::Step(SocialInput &input, SocialOutput &output) {
  CheckStatus(input, output.status);
  if (output.status == NavState::RESUME) {
    auto reset = (prev_nav_state_ != NavState::RESUME &&
                  prev_nav_state_ != NavState::ALMOST);
    metrics_calculator_->Update(input, reset);
    std::vector<float> state{};
    state.reserve(SOCIAL_STATE_SIZE);
    GenerateState(input, state);
    GetAction(state, output);
  } else if (output.status == NavState::ALMOST) {
    output.wp_x = input.target.x;
    output.wp_y = input.target.y;
    output.apf_k = kMaxApfK;
    output.apf_beta_k = 0.0;
  } else {
    output.wp_x = 0.0;
    output.wp_y = 0.0;
    output.apf_k = 1.0;
    output.apf_beta_k = 0.0;
  }

  prev_nav_state_ = output.status;
}

void SocialApfRl::GenerateState(SocialInput &input, std::vector<float> &state) {

  // std::for_each(input.scan.begin(), input.scan.end(), [](double& d) {
  // d-=kRhoRobot;});
  for (auto &range : input.scan) {
    range -= kRhoRobot;
  }

  if (is_initial_) {
    state.insert(state.end(), input.scan.begin(), input.scan.end());
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
    int shift =
        static_cast<int>((SOCIAL_STATE_SCAN_SIZE * yaw_diff) / (2.0 * M_PI));
    // np.roll(old_scan, shift) in the trainer moves elements towards HIGHER
    // indices; std::rotate(first, first+n, last) moves them towards LOWER ones.
    // Rotate left by (N - shift) to reproduce np.roll. See apfrl.cpp.
    shift = ((shift % SOCIAL_STATE_SCAN_SIZE) + SOCIAL_STATE_SCAN_SIZE) %
            SOCIAL_STATE_SCAN_SIZE;
    const int rot = (SOCIAL_STATE_SCAN_SIZE - shift) % SOCIAL_STATE_SCAN_SIZE;
    std::array<float, SOCIAL_STATE_SCAN_SIZE> old_scan;
    std::copy(prev_state_.begin(), prev_state_.begin() + SOCIAL_STATE_SCAN_SIZE,
              old_scan.begin());

    (void)std::rotate(old_scan.begin(), old_scan.begin() + rot,
                      old_scan.end());
    state.insert(state.end(), old_scan.begin(), old_scan.end());
    state.push_back(prev_state_[SOCIAL_STATE_SCAN_SIZE]);
    state.push_back(prev_state_[SOCIAL_STATE_SCAN_SIZE + 1]);
  }
  auto classes = CreateScanClasses(input.scan, input.humans);
  state.insert(state.end(), input.scan.begin(), input.scan.end());
  state.push_back(input.target.x);
  state.push_back(input.target.y);


  state.insert(state.end(), classes.begin(), classes.end());
  prev_state_ = {};
  std::copy(state.begin() + (SOCIAL_STATE_SCAN_SIZE + 2),
            state.begin() + (2 * SOCIAL_STATE_SCAN_SIZE + 4),
            prev_state_.begin());

  // Normalise and clip as the trainer does -- np.clip(raw/(laser_max/2), -1, 1)
  // over the lidar half only; the 0/1 class block stays unscaled.
  for (unsigned int i = 0; i < 2 * SOCIAL_STATE_SCAN_SIZE + 4; i++) {
    state[i] /= kNormalizationVal;
    state[i] = std::max(-1.0f, std::min(1.0f, state[i]));
  }
}

void SocialApfRl::GetAction(std::vector<float> &state, SocialOutput &output) {
  torch::NoGradGuard no_grad;
  std::vector<torch::jit::IValue> inputs{};
  torch::Tensor state_tensor =
      torch::from_blob(state.data(), {1, SOCIAL_STATE_SIZE}).to(at::kCUDA);
  inputs.emplace_back(state_tensor);
  at::Tensor actionT = module_.forward(inputs).toTensor().to(
      torch::kCPU); // get action using model
  std::vector<float> action(actionT.data_ptr<float>(),
                            actionT.data_ptr<float>() + actionT.numel());
  output.wp_x = (action[0] + 1.0) * kXScale / 2;


  output.wp_y = action[1] * kYScale;
  // output.wp_y = 0.5*prev_gy_ + (1-0.5)* output.wp_y;
  // prev_gy_ = output.wp_y;
  output.apf_k = (action[2] + 1.5) * 3.0;
  // 4th action — current convention. mu drives the obstacle-enlargement-factor
  // function on the planner side. Mapping (action[3]+1)/2 -> [0, 1] mirrors
  // the previous beta_k mapping for symmetry; tune on the planner side.
  output.mu = (action[3] + 1) / 2;
  // beta_k preserved-but-zero — see SocialOutput in data_structures.h.
  output.apf_beta_k = 0.0;

//  bool human_ahead = (state[2 * SOCIAL_STATE_SCAN_SIZE + 4] == 1.0 &&
//                      state[SOCIAL_STATE_SCAN_SIZE + 2] < 2.5) ||
//                     state[2 * SOCIAL_STATE_SCAN_SIZE + 5] == 1.0 &&
//                         (state[SOCIAL_STATE_SCAN_SIZE + 3] < 2.5) ||
//                     (state[SOCIAL_STATE_SIZE - 1] == 1.0 &&
//                      state[2 * SOCIAL_STATE_SCAN_SIZE + 1] < 2.5) ||
//                     (state[SOCIAL_STATE_SIZE - 2] == 1.0 &&
//                      state[2 * SOCIAL_STATE_SCAN_SIZE] < 2.5);
//
//  if (min_human_dist_ < kHumanClassDistThreshold) {
//    //output.wp_x = 1.0;std::min(output.wp_x, 0.2);
//    output.apf_k = std::min(output.apf_k, 2.5);
//  }
}

void SocialApfRl::CheckStatus(const SocialInput &input, NavState &status) {
  bool collision = *std::min_element(input.scan.begin(), input.scan.end()) <
                   kCollisionDistance;
  auto goal_distance = std::pow(
      (std::pow(input.target.x, 2) + std::pow(input.target.y, 2)), 0.5);
  bool reached = goal_distance < kReachDistance;
  bool almost = goal_distance < kCloseDistance;
  bool wait = (input.target.x == 0.0 && input.target.y == 0.0);

  if (wait) {
    status = NavState::WAIT;
  } else if (reached) {
    status = NavState::SUCCESS;
    std::cout << "SUCCESS!!" << std::endl;
    metrics_calculator_->PrintMetrics();
  } else if (almost) {
    status = NavState::ALMOST;
  } else if (collision) {
    status = NavState::COLLISION;
    std::cout << "COLLISION!!" << std::endl;
    metrics_calculator_->PrintMetrics();
  } else {
    status = NavState::RESUME;
  }
}

std::vector<float>
SocialApfRl::CreateScanClasses(std::vector<double> &scan,
                               const std::vector<Pose> &humans) {
  std::vector<float> classes{};
  classes.resize(SOCIAL_STATE_SCAN_SIZE, 0.0);
  for (auto &human : humans) {
    auto dist = sqrt(human.x * human.x + human.y * human.y);
    auto theta = atan2(human.y, human.x);
    auto idx =
        Modulo(int(round(SOCIAL_STATE_SCAN_SIZE * theta / (2.0 * M_PI))));
    if (dist < kHumanClassDistThreshold) {
      classes[idx] = 1.0;
      classes[Modulo(idx + 1)] = 1.0;
      classes[Modulo(idx - 1)] = 1.0;
      scan[idx] = std::min(dist -0.6 , scan[idx]);
      scan[Modulo(idx + 1)] = std::min(dist-0.6, scan[Modulo(idx + 1)]);
      scan[Modulo(idx - 1)] = std::min(dist-0.6, scan[Modulo(idx - 1)]);
    } else {
      classes[idx] = 1.0;
      scan[idx] = std::min(dist-0.6, scan[idx]);
    }
  }

  return classes;
}
double SocialApfRl::subtract_radius(double x) { return (x - kRhoRobot); }
double SocialApfRl::normalize(double x) { return (x / kNormalizationVal); }
