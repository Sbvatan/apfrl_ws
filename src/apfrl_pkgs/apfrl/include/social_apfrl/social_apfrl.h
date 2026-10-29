#ifndef APFRL_INCLUDE_SOCIAL_APFRL_H_
#define APFRL_INCLUDE_SOCIAL_APFRL_H_
#include <cmath>
#include "data_structures.h"
#include <torch/script.h>
#include <algorithm>
#include "metrics_calculator.h"
#include <memory>

/// @brief Calculates inputs for Apf, consists of waypoint and k parameter
class SocialApfRl
{
public:
  explicit SocialApfRl(std::string &model_path);
  /// @brief Main function of the class. Creates input tensor and uses rl model to calculate outputs
  /// @param input Inputs consist of scan data and target location
  /// @param output Outputs consist of waypoint and k parameter
  void Step(SocialInput &input, SocialOutput &output);

private:
  /// @brief Converts inputs to state format that RL expects
  /// @param input Inputs consist of scan data and target location
  /// @param state Output state to use in RL model
  void GenerateState(SocialInput &input, std::vector<float> &state);
  /// @brief Uses calculated state to get action using RL model
  /// @param state Input
  /// @param output Outputs consist of waypoint and k parameter
  void GetAction(std::vector<float> &state, SocialOutput &output);
  /// @brief Checks status of the navigation task
  /// @param input Inputs consist of scan data and target location
  /// @param status status of the navigation task(success, collision etc.)
  void CheckStatus(const SocialInput &input, NavState &status);
  /// @brief Subtracts robot radius from the scan data
  static double subtract_radius(double x);
  /// @brief Normalizes data by dividing to kNormalizationVal
  static double normalize(double x);

  std::vector<float> CreateScanClasses(std::vector<double> &scan, const std::vector<Pose> &humans);
  torch::jit::script::Module module_;
  std::array<float, SOCIAL_STATE_SCAN_SIZE + 2> prev_state_;
  double prev_yaw_;
  bool is_initial_;
  NavState prev_nav_state_;
  std::unique_ptr<MetricsCalculator> metrics_calculator_;

  static constexpr double kRhoRobot = 0.4; // case 2-> 0.275, case3->0.35 ,orj:0.4
  // Deploy half of the contract with SocialAPFRL_env_Ata.py: laser_max 150,
  // norm laser_max/2 = 75, x_scale = y_scale = 2.5*r_robot. See CODE_REVIEW.md.
  static constexpr double kNormalizationVal = 75.0;  // env laser_max / 2
  static constexpr double kXScale = 2.5 * kRhoRobot;
  static constexpr double kYScale = 2.5 * kRhoRobot;
  static constexpr double kCollisionDistance = kRhoRobot + 0.05; //-0.3 to eliminate collision
  static constexpr double kReachDistance = 0.2;
  static constexpr double kCloseDistance = 0.6; // orj: 0.6 case 3-> 1.2
  static constexpr double kMaxApfK = 6.0;
  static constexpr double kHumanClassDistThreshold = 2.0;
};

inline unsigned Modulo(int value, unsigned m = SOCIAL_STATE_SCAN_SIZE)
{
  int mod = value % (int)m;
  if (mod < 0)
  {
    mod += m;
  }
  return mod;
}
#endif // APFRL_INCLUDE_SOCIAL_APFRL_H_
