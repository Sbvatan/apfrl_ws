#include "metrics_calculator.h"

MetricsCalculator::MetricsCalculator() {
  metrics_ = {};
  prev_pose_ = {};
  init_time_ = rclcpp::Clock{RCL_ROS_TIME}.now().seconds();

}
void MetricsCalculator::Update(SocialInput &input, bool reset) {
  if (reset){
    metrics_ = {};
    init_time_ = rclcpp::Clock{RCL_ROS_TIME}.now().seconds();
    prev_pose_.x = input.robot_pose.x;
    prev_pose_.y = input.robot_pose.y;
  }
  metrics_.time = rclcpp::Clock{RCL_ROS_TIME}.now().seconds() - init_time_;
  metrics_.dist += CalcDistBetween(input.robot_pose, prev_pose_);
  prev_pose_ = input.robot_pose;
  for (const auto &human: input.humans){

    metrics_.human_dists.push_back(CalcHumanDist(human));
  }


}

void MetricsCalculator::PrintMetrics() {
  std::cout << "Travel Time: " << metrics_.time << std::endl;
  std::cout << "Travel Dist: " << metrics_.dist << std::endl;
  if (!metrics_.human_dists.empty()){
  std::cout << "Min Human Dist: " << *min_element(metrics_.human_dists.begin(), metrics_.human_dists.end()) << std::endl;
  std::cout << "Mean Human Dist: " << std::accumulate(metrics_.human_dists.begin(), metrics_.human_dists.end(), 0.0) / metrics_.human_dists.size() << std::endl;
}
}


double MetricsCalculator::CalcHumanDist(const Pose &human){
   return std::sqrt(human.x * human.x + human.y * human.y);
}

double MetricsCalculator::CalcDistBetween(const Pose &pose1, const Pose &pose2){
  auto dx = pose1.x - pose2.x;
  auto dy = pose1.y - pose2.y;
  return std::sqrt(dx * dx + dy * dy);
}
