//#include <obs_generator/obs_generator.h>
//#include <ros/ros.h>

#include "obs_generator/obs_generator.h"

#include <fstream>
#include <iostream>

void ObsGenerator::setLaserParams(const LaserParams &params) {
    maxRangeReading = params.range_max;
    kLaserScanIncRad = params.angle_increment;
    kLaserMinAngle = params.angle_min;
    kLaserPanAngleHalfRad = params.panAngleHalfRad();
    scan_size_ = params.scan_size;
}

void ObsGenerator::modelObstaclesAsEllipses(const Input &input,
                                            std::vector<Obstacle> &obstacles) {
  std::vector<Point> points;

  GeneratePoints(input.scan, points);
  /*
      std::ofstream scan_file;
      scan_file.open ("scan_output.txt");
      for (int k = 1; k< input.scan.size(); k++){
      double th = ( k)*kLaserScanIncRad - kLaserPanAngleHalfRad;
      //scan_file<<k<<" "<<input.scan[k]<<" "<<th<<" "<<points[k].x<<"
     "<<points[k].y<<" "<<input.scan[k]-input.scan[k-1]<<std::endl;

      }
    */

  std::vector<RangeCluster> clusters;
  ClusterScanAlperen(input.scan, points, -kLaserPanAngleHalfRad,
                     kLaserPanAngleHalfRad, clusters);

  std::vector<bool> humans_added;
  humans_added.resize(input.humans.size(), false);

  double eps = 0.01;
  double lmcoeff = 1e-8;
  for (auto &cluster : clusters) {
    if (cluster.points.size() >= kMinClusterSize) {
      Obstacle obs_temp = EllipseFit::computeAlperen(cluster.points, epsilon);
      obstacles.push_back(obs_temp);
    }
  }

  for (unsigned int i = 0; i < input.humans.size(); i++) {
    if (!humans_added[i]) {
      Obstacle obs_temp{};
      obs_temp.x = input.humans[i].x;
      obs_temp.y = input.humans[i].y;
      obs_temp.yaw = atan2(input.humans[i].v_y, input.humans[i].v_x);
      auto v = sqrt(input.humans[i].v_x * input.humans[i].v_x +
                    input.humans[i].v_y * input.humans[i].v_y);
      if (v > 0.2) {
        obs_temp.r_a = kHumanEllipseRA + input.human_coef * v;
        obs_temp.r_b = kHumanEllipseRB + input.human_coef * v / 2.0;
      } else {
        obs_temp.r_a = kHumanEllipseRA + input.human_coef * 0.35;
        obs_temp.r_b = kHumanEllipseRA + input.human_coef * 0.35;
      }

      obs_temp.x =
          -input.humans[i].x + input.human_coef * v * cos(obs_temp.yaw) / 2.;
      obs_temp.y =
          -input.humans[i].y + input.human_coef * v * sin(obs_temp.yaw) / 2.;
      obstacles.push_back(obs_temp);
    }
  }
}

void ObsGenerator::generate(const Input &input,
                            std::vector<Obstacle> &obstacles) {
  std::vector<Point> points;

  GeneratePoints(input.scan, points);

  std::vector<std::vector<Point>> clusters;
  ClusterScan(input.scan, points, -kLaserPanAngleHalfRad, kLaserPanAngleHalfRad,
              clusters);

  int cnt = 0;
  for (auto &cluster : clusters) {
    // Next cluster
    cnt++;
    for (int k = 0; k < cluster.size(); k++) {
      std::cout << cnt << " " << cluster[k].x << " " << cluster[k].y
                << std::endl;
    }
  }

  std::vector<bool> humans_added;
  humans_added.resize(input.humans.size(), false);

  for (auto &cluster : clusters) {
    if (cluster.size() >= kMinClusterSize)

    {
      Obstacle obs_temp{};
      EllipseFit::fit(cluster, obs_temp);

      MatchToHuman(obs_temp, humans_added, input);
      obstacles.push_back(obs_temp);
    }
  }

  for (unsigned int i = 0; i < input.humans.size(); i++) {
    if (!humans_added[i]) {
      Obstacle obs_temp{};
      obs_temp.x = input.humans[i].x;
      obs_temp.y = input.humans[i].y;
      obs_temp.yaw = atan2(input.humans[i].v_y, input.humans[i].v_x);
      auto v = sqrt(input.humans[i].v_x * input.humans[i].v_x +
                    input.humans[i].v_y * input.humans[i].v_y);
      if (v > 0.2) {
        obs_temp.r_a = kHumanEllipseRA + input.human_coef * v;
        obs_temp.r_b = kHumanEllipseRB + input.human_coef * v / 2.0;
      } else {
        obs_temp.r_a = kHumanEllipseRA + input.human_coef * 0.35;
        obs_temp.r_b = kHumanEllipseRA + input.human_coef * 0.35;
      }

      obs_temp.x =
          -input.humans[i].x + input.human_coef * v * cos(obs_temp.yaw) / 2.;
      obs_temp.y =
          -input.humans[i].y + input.human_coef * v * sin(obs_temp.yaw) / 2.;
      obstacles.push_back(obs_temp);
    }
  }
}

void ObsGenerator::MatchToHuman(Obstacle &obs, std::vector<bool> &humans_added,
                                const Input &input) {
  auto min_size = 0.15;
  for (int i = 0; i < input.humans.size(); i++) {
    auto dx = -input.humans[i].x - obs.x;
    auto dy = -input.humans[i].y - obs.y;
    auto dist = sqrt(dx * dx + dy * dy);
    auto vel_theta = atan2(input.humans[i].v_y, input.humans[i].v_x);
    if (dist < kMaxHumanEllipseDist) {
      obs.r_a = std::max(obs.r_a, min_size);
      obs.r_b = std::max(obs.r_b, min_size);
      auto v = sqrt(input.humans[i].v_x * input.humans[i].v_x +
                    input.humans[i].v_y * input.humans[i].v_y);
      auto th = vel_theta - obs.yaw;
      if (th > -M_PI / 4 && th < M_PI / 4) {
        obs.r_a += input.human_coef * v;
        obs.r_b += input.human_coef * v / 2;
        obs.x += v * cos(obs.yaw) / 2.;
        obs.y += v * sin(obs.yaw) / 2.;
      } else if (th > 3 * M_PI / 4 && th < 5 * M_PI / 4) {
        obs.r_a += input.human_coef * v;
        obs.r_b += input.human_coef * v / 2;
        obs.x += v * cos(obs.yaw + M_PI) / 2.;
        obs.y += v * sin(obs.yaw + M_PI) / 2.;
      } else if (th > M_PI / 4 && th < 3 * M_PI / 4) {
        obs.r_b += input.human_coef * v;
        obs.r_a += input.human_coef * v / 2;
        obs.x -= v * sin(obs.yaw) / 2.;
        obs.y += v * cos(obs.yaw) / 2.;
      } else {
        obs.r_b += input.human_coef * v;
        obs.r_a += input.human_coef * v / 2;
        obs.x -= v * sin(obs.yaw + M_PI) / 2.;
        obs.y += v * cos(obs.yaw + M_PI) / 2.;
      }
      humans_added[i] = true;
      break;
    }
  }
}

/*
 *  This method clusters the input laser data
 *
 *
 *
 *
 */
void ObsGenerator::ClusterScan(const std::vector<double> &scan,
                               const std::vector<Point> &points,
                               const double startPan, const double endPan,
                               std::vector<std::vector<Point>> &clusters) {
  unsigned int count = 1U;
  std::vector<int> start_idx = {0};
  std::vector<int> end_idx = {0};
  end_idx = {0U};

  double accRange = 0;
  double clusterCnt = 0;
  const int scan_sz = static_cast<int>(scan.size());
  for (unsigned int i = 1U; i < scan_sz - 1U; i++) {
    // Look at the difference btw consecutive range readings
    double diff = fabs(scan[i + 1U] - scan[i]);

    double derDiff =
        fabs(fabs(scan[i + 1U] - scan[i]) - fabs(scan[i] - scan[i - 1U]));

    // If the range reading is large
    // if ( diff> kClusterDistThreshold)

    double d = std::pow(scan[i + 1U], 2) + std::pow(scan[i], 2) -
               2.0 * scan[i + 1U] * scan[i] * cos(kLaserScanIncRad);
    d = std::sqrt(d);

    if ((derDiff > kClusterDistThreshold) || (d > minGapDist)) {
      count++;
      start_idx.push_back(i + 1U);
      end_idx.push_back(i + 1U);
    } else {
      end_idx[count - 1U] = i;
    }
  }

  if (end_idx[end_idx.size() - 1U] > (scan_sz - kLaserClusterPar)) {
    start_idx[0U] = start_idx[start_idx.size() - 1U] - scan_sz;
    (void)start_idx.erase(start_idx.end() - 1);
    (void)end_idx.erase(end_idx.end() - 1);
  }
  // ROS_INFO("ObsGenerator::ClusterScan 3");
  // Merge last and first clusters
  for (unsigned int i = 0U; i < start_idx.size(); ++i) {
    std::vector<Point> temp_cluster{};
    if (start_idx[i] < 0U && end_idx[i] < 0U) {
      start_idx[i] += scan_sz;
      end_idx[i] += scan_sz;
    }

    // If the cluster has been merged
    if (start_idx[i] < 0) {
      temp_cluster = std::vector<Point>(
          points.begin() + start_idx[i] + scan_sz, points.end());

      (void)temp_cluster.insert(temp_cluster.end(), points.begin(),
                                points.begin() + end_idx[i]);
    } else {
      temp_cluster = std::vector<Point>(points.begin() + start_idx[i],
                                        points.begin() + end_idx[i]);
    }
    clusters.push_back(temp_cluster);
  }
}

/*
 *  This method clusters the input laser data
 *
 *
 *
 *
 */
void ObsGenerator::ClusterScanWrtDiscontinuities(
    const std::vector<double> &scan, const std::vector<Point> &points,
    const double startPan, const double endPan,
    std::vector<RangeCluster> &clusters) {
  unsigned int count = 1U;
  std::vector<int> start_idx = {0};
  std::vector<int> end_idx = {0};
  end_idx = {0U};

  double gap = endPan - startPan;

  const int scan_sz = static_cast<int>(scan.size());
  // start_idx and end_idx contain indeces where depth jump >
  for (unsigned int i = 0; i < scan_sz - 1; i++) {
    // Look at the difference btw consecutive range readings
    double diff = fabs(scan[i + 1U] - scan[i]);

    double derRange = fabs((scan[i + 1U] - scan[i]) - (scan[i] - scan[i - 1U]));

    // If the range reading is large
    double d = std::pow(scan[i + 1U], 2) + std::pow(scan[i], 2) - 2.0 * scan[i + 1U] * scan[i] * cos(kLaserScanIncRad);
    d = std::sqrt(d);

    // Start a new cluster
    if ((derRange > maxRangeDerivative) || (d > minGapDist))

    // If the range reading is large
    {
      count++;
      start_idx.push_back(i + 1U);
      end_idx.push_back(i + 1U);
    } else {
      end_idx[count - 1U] = i;
    }
  }

  for (unsigned int i = 0U; i < start_idx.size(); ++i) {
    std::vector<Point> temp_cluster{};
    RangeCluster rangeCluster;
    double avgLaserReading = 0;

    for (int k = start_idx[i]; k <= end_idx[i]; k++) avgLaserReading += scan[k];

    for (int k = start_idx[i]; k <= end_idx[i]; k++)
      temp_cluster.push_back(points.at(k));

    avgLaserReading /= temp_cluster.size();

    if ((avgLaserReading <= maxRangeReading) &&
        (temp_cluster.size() >= kMinClusterSize)) {
      rangeCluster.points = temp_cluster;
      clusters.push_back(rangeCluster);
    }
  }
}

void ObsGenerator::ClusterScanAlperen(const std::vector<double> &scan,
                                      const std::vector<Point> &points,
                                      const double startPan,
                                      const double endPan,
                                      std::vector<RangeCluster> &clusters) {
  double dmax = 0;
  double d = 0;
  int last_idx = 0;

  // start_idx and end_idx contain indeces where depth jump >
  // kClusterDistThreshold
  for (unsigned int i = 1; i < points.size(); i++) {
    double dmax = points[i - 1].range * sin(kLaserScanIncRad) /
                  sin(lambda - kLaserScanIncRad);
    double d = pow((points[i].x - points[i - 1].x), 2) +
               pow((points[i].y - points[i - 1].y), 2);

    if (sqrt(d) > dmax + 3 * kLaserNoiseStd) {
      std::vector<RangeCluster> tempCluster = DouglasPeucker(
          std::vector<Point>(points.begin() + last_idx, points.begin() + i),
          epsilon);
      clusters.insert(clusters.end(), tempCluster.begin(), tempCluster.end());
      last_idx = i;
    }
  }
  std::vector<RangeCluster> tempCluster = DouglasPeucker(
      std::vector<Point>(points.begin() + last_idx, points.end()), epsilon);
  clusters.insert(clusters.end(), tempCluster.begin(), tempCluster.end());
}

std::vector<RangeCluster> ObsGenerator::DouglasPeucker(
    std::vector<Point> PointList, float epsilon) {
  double dmax = 0;
  unsigned int index = 0;
  int end = PointList.size();

  double A = PointList[end - 1].y - PointList[0].y;
  double B = PointList[0].x - PointList[end - 1].x;
  double C = PointList[end - 1].x * PointList[0].y -
             PointList[0].x * PointList[end - 1].y;

  for (int i = 2; i < end - 1; i++) {
    double d = distanceFromLine(PointList[i], A, B, C);

    if (d > dmax) {
      dmax = d;
      index = i;
    }
  }

  std::vector<RangeCluster> ResultList;

  if (dmax > epsilon) {
    std::vector<RangeCluster> recResult1 = DouglasPeucker(
        std::vector<Point>(PointList.begin(), PointList.begin() + index + 1),
        epsilon);
    std::vector<RangeCluster> recResult2 = DouglasPeucker(
        std::vector<Point>(PointList.begin() + index, PointList.end()),
        epsilon);

    ResultList.insert(ResultList.end(), recResult1.begin(), recResult1.end());
    ResultList.insert(ResultList.end(), recResult2.begin(), recResult2.end());
  } else {
    std::vector<RangeCluster> recResult;
    ResultList.push_back(
        RangeCluster{std::vector<Point>(PointList.begin(), PointList.end())});
  }

  return ResultList;
}

double ObsGenerator::distanceFromLine(Point point1, double A, double B,
                                      double C) {
  double distance =
      std::abs(A * point1.x + B * point1.y + C) / std::sqrt(A * A + B * B);
  return distance;
}

void ObsGenerator::GeneratePoints(const std::vector<double> &scan,
                                  std::vector<Point> &points) {
  for (unsigned int i = 0; i < scan.size(); i++) {
    if (scan[i] < maxRangeReading && scan[i] > 0) {
      double th = (i) * kLaserScanIncRad + kLaserMinAngle;
      points.emplace_back(scan[i] * cos(th), scan[i] * sin(th), th, scan[i]);
    }
  }
}
