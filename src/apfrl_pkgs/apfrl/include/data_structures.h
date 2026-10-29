#ifndef APFRL_DATA_STRUCTURES_H_
#define APFRL_DATA_STRUCTURES_H_

#define SCAN_SIZE 3200
#define STATE_SCAN_SIZE 64
#define SCAN_STEP 50 //1024/64
#define STATE_SIZE 132

// Social variant uses a coarser decimation than basic apfrl: 1024 → 32 rays.
// State layout (3*32 + 4 = 100 floats):
//   [old_scan(32), old_target(2), curr_scan(32), curr_target(2), classes(32)]
// "classes" is a 0/1 mask that flags scan rays containing a human (from
// /humans — actors_pub in sim, tracker_node in real life).
//
// 32 is the design size, not an arbitrary one: models/social_apfrl_model.pt
// takes exactly 100 inputs. A previous pass bumped this to 64 (196 floats) to
// match a training env that had itself been widened; that combination cannot
// load the shipped policy at all -- torch::from_blob hands the module a 196-wide
// tensor and the first Linear (100→512) throws. Keep this in step with
// SocialAPFRL_env_Ata.py's scan_size.
#define SOCIAL_STATE_SCAN_SIZE 32
#define SOCIAL_SCAN_STEP 32 //1024/32
#define SOCIAL_STATE_SIZE 100

#include <vector>
#include <array>
#include <string>

enum NavState{
  WAIT =      0,
  RESUME  =   1,
  SUCCESS =   2,
  COLLISION=  3,
  ALMOST=     4,
};

struct Pose {
  double x;
  double y;
  double z;
  double yaw;
  std::string frame;
};

struct Input {
  std::array<double,STATE_SCAN_SIZE> scan;
  Pose target;
  Pose robot_pose;
};

struct Output {
  double apf_k;
  double wp_x;
  double wp_y;
  NavState status;
};

struct SocialInput {
  std::vector<double> scan;
  std::vector<Pose> humans;
  Pose target;
  Pose robot_pose;
};

struct SocialOutput {
  double apf_k;
  double wp_x;
  double wp_y;
  // 4th action — current convention. Drives the obstacle-enlargement-factor
  // function on the planner side. Published on /apfrl_mu by social_apfrl::ros_comm.
  double mu;
  // Deprecated 4th action — preserved for backwards-compat with older code
  // paths that still publish on /apf_beta_k. Not written by GetAction in
  // the current trainer; kept zero. To be reused later for a 5th action.
  double apf_beta_k;
  NavState status;
};

struct Metrics {
  double time;
  double dist;
  std::vector<double> human_dists;
};

struct Obstacle {
    double x;
    double y;
    double yaw;
    double r_a;
    double r_b;
};

struct Velocity {
    double linear;
    double angular;
};

struct ApfInput {
    std::vector<Obstacle> obstacles;
    std::vector<Obstacle> ultrasonic_obstacles;
    Pose target;
    float apf_k;
    //float apf_beta_k;
};

#endif // APFRL_DATA_STRUCTURES_H_
