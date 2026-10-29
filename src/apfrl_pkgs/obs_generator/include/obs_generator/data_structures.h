#ifndef OBS_GENERATOR_DATA_STRUCTURES_H_
#define OBS_GENERATOR_DATA_STRUCTURES_H_

#include <vector>
#include <array>
#include <cmath>


#define DEBUG_MODE 0
#define IS_ALPEREN 1

/// Laser parameters extracted from sensor_msgs::msg::LaserScan
struct LaserParams
{
  double angle_min = -M_PI;       // from scan_msg->angle_min
  double angle_max = M_PI;        // from scan_msg->angle_max
  double angle_increment = 0.0;   // from scan_msg->angle_increment
  double range_max = 40.0;        // from scan_msg->range_max
  size_t scan_size = 0;           // from scan_msg->ranges.size()

  double panAngleHalfRad() const { return (angle_max - angle_min) / 2.0; }
  bool isValid() const { return scan_size > 0 && angle_increment > 0.0; }
};
struct Point
{
  double x;
  double y;
  double th;
  double range;

  bool operator==(const Point &p)
  {
    return (x == p.x && y == p.y);
  }
  bool operator>(const Point &p)
  {
    if (x != p.x)
      return x > p.x;
    else
      return y > p.y;
  }
  Point(double x, double y, double th, double range) : x(x), y(y), th(th), range(range){};
  Point() : x(0), y(0), th(0), range(0){};
};
struct Obstacle
{
  double x;
  double y;
  double yaw;
  double r_a;
  double r_b;
};
struct Human
{
  double x;
  double y;
  double v_x;
  double v_y;

  bool operator=(const Human &h)
  {
    x = h.x;
    y = h.y;
    v_x = h.v_x;
    v_y = h.v_y;
    return true;
  }
};
struct Input
{
  std::vector<double> scan;
  std::vector<Human> humans;
  double human_coef;
  LaserParams laser_params;
};

/// Algorithm output: ellipse-modelled obstacles in the laser frame.
/// RosComm transforms them to base_frame before publishing.
struct Output
{
  std::vector<Obstacle> obstacles;
};

#ifdef IS_ALPEREN

struct  RangeCluster
{

    std::vector<Point> points;

};
#else
struct  RangeCluster
{
  int startIndex;
  int endIndex;
  double startPanAngleRad;
  double endPanAngleRad;
  std::vector<Point> points;
  double avgRangeReading;
};

#endif

struct dataPt
{
    int index;
    double r;
    double f1;
    double x;
    double y;
};

#endif // OBS_GENERATOR_DATA_STRUCTURES_H_
