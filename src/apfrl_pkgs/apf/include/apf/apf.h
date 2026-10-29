#ifndef APF_H
#define APF_H

#include <cmath>
#include "data_structures.h"


/// @brief Artificial Potential Fields
class APF
{
public:
  APF(double xScale);
  /// @brief Calculates velocity output using APF formulation
  /// @param input Inputs consist of target, obstacles and k parameter
  /// @param output Outputs consist of linear and velocity
  void CalculateVel(const Input &input, Velocity &output);
  double getMinBetaCurrent();

private:
  static constexpr double kRobotRadius = 0.4; // 0.4; 0.45
  double kXscale;
  static constexpr double kMaxV = 0.2;               // 0.8; 0.5
  static constexpr double kMinV = 0.0;              // -0.3; 0.2
  static constexpr double kMaxW = M_PI / 8;          // Original: M_PI/4; M_PI/3
  static constexpr double kSafetyFactor = 1.2;       // 1.2;  1.337
  static constexpr double kOuterSafetyFactor = 1.20; // 1.1; 1.62
  static constexpr double kMinGamma = 0.02;         // 0.02;
  double minBetaCurrent_ = 0.0;                      // To keep track of minimum beta
  double Beta_min_ = 0.01;                           // Threshold for obstacle beta
};
#endif // APF_H
