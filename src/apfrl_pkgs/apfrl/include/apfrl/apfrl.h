#ifndef APFRL_INCLUDE_APFRL_H_
#define APFRL_INCLUDE_APFRL_H_
#include <cmath>
#include <torch/script.h>
#include <torch/torch.h>
#include <algorithm>
#include <memory>
#include "data_structures.h"
#include "metrics_calculator.h"



/// @brief Which observation contract a policy was trained under.
///
/// The weights decide this, not the robot. Policies trained before the BARN
/// work saw a 150 m clamp and one shared divisor of 75 for both scan and goal;
/// the BARN policies saw a 28 m clamp, an affine scan and a separate goal norm
/// of 30. Feeding either policy the other's scaling is a silent train/deploy
/// mismatch -- the network still runs, it just receives an input distribution
/// it never saw. See CODE_REVIEW.md.
enum class ObsProfile {
  kBarn,    ///< clamp 28, scan 2r/28-1, goal /30 scaled by magnitude
  kLegacy,  ///< clamp 150, scan and goal both /75, clipped per component
};

/// @brief Parse a profile name; throws std::invalid_argument on anything else.
ObsProfile ParseObsProfile(const std::string &name);

/// @brief Calculates inputs for Apf, consists of waypoint and k parameter
class ApfRl {
public:
  explicit ApfRl(std::string &model_path, ObsProfile profile = ObsProfile::kBarn);
  /// @brief The clamp this profile expects on a raw lidar range, in metres.
  static double MaxRangeFor(ObsProfile profile);
  /// @brief Main function of the class. Creates input tensor and uses rl model to calculate outputs
  /// @param input Inputs consist of scan data and target location
  /// @param output Outputs consist of waypoint and k parameter
  void Step(SocialInput &input, Output &output);

private:
  /// @brief Converts inputs to state format that RL expects
  /// @param input Inputs consist of scan data and target location
  /// @param state Output state to use in RL model
  void GenerateState(SocialInput& input, std::vector<float> &state);
  /// @brief Uses calculated state to get action using RL model
  /// @param state Input
  /// @param output Outputs consist of waypoint and k parameter
  void GetAction(std::vector<float> &state, Output &output);
  /// @brief Checks status of the navigation task
  /// @param input Inputs consist of scan data and target location
  /// @param status status of the navigation task(success, collision etc.)
  void CheckStatus(const SocialInput &input, NavState &status);
  /// @brief Subtracts robot radius from the scan data
  static double subtract_radius(double x);
  /// @brief Normalizes data by dividing to kScanNorm
  static double normalize(double x);

  torch::jit::script::Module module_;
  std::array<float,STATE_SIZE/2> prev_state_;
  double prev_yaw_;
  bool is_initial_;
  NavState prev_nav_state_;
  std::unique_ptr<MetricsCalculator> metrics_calculator_;

  // The four constants below are the deploy half of the observation/action
  // contract with the trainer. They must track APFRL_env_Ata.py (and the
  // identical STSAC env), because that is what the weights in models/*.pt were
  // fitted on:
  //     r_robot 0.4   laser_max 28   scan 2*r/28-1   goal_norm 30
  //     x/y_scale 2.5*r
  //
  // Scan and goal are mapped differently on purpose. The scan is affine onto
  // the full [-1, 1] -- a distance is non-negative, so a pure division can
  // never reach [-1, 0). The goal is divided and scaled by magnitude so it
  // does not saturate: a catalogue traverse starts ~18.2 m out.
  // Changing one end without the other feeds the policy an input distribution
  // it never saw. See CODE_REVIEW.md.
  static constexpr double kRhoRobot = 0.4;      // env r_robot; also the lidar <min>
  static constexpr double kScanNorm = 28.0;     // env laser_max; scan is 2*r/kScanNorm - 1
  static constexpr double kGoalNorm = 30.0;     // env goal_norm
  // The pre-BARN contract, kept so apfrl_model.pt and stackelberg_model.pt can
  // still be deployed against the scaling they were actually fitted on.
  static constexpr double kLegacyNorm = 75.0;      // old kNormalizationVal, laser_max/2
  static constexpr double kLegacyMaxRange = 150.0; // old ros_comm kMaxRange
  static constexpr double kBarnMaxRange = 28.0;

  ObsProfile profile_;
  static constexpr double kXScale = 2.5*kRhoRobot;
  static constexpr double kYScale = 2.5*kRhoRobot;
  static constexpr double kCollisionDistance = kRhoRobot -0.1; //-0.3 for collusion removal
  static constexpr double kReachDistance = 0.2; //apfrl 0.2, sfm 0.4
  static constexpr double kCloseDistance = 0.6; // orj:0.6
  static constexpr double kMaxApfK = 5.0;
};
#endif // APFRL_INCLUDE_APFRL_H_
