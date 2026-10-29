#ifndef APFRL_INCLUDE_METRICS_H_
#define APFRL_INCLUDE_METRICS_H_
#include <cmath>
#include "data_structures.h"
#include <algorithm>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <numeric>
/// @brief Calculates inputs for Apf, consists of waypoint and k parameter
class MetricsCalculator{
public:
  explicit MetricsCalculator();

  void Update(SocialInput &input, bool reset = false);

  void PrintMetrics();

private:
  Metrics metrics_;
  double init_time_;
  Pose prev_pose_;

  static double CalcHumanDist(const Pose &human);
  static double CalcDistBetween(const Pose &pose1, const Pose &pose2);
};


#endif // APFRL_INCLUDE_METRICS_H_
