#ifndef APF_DATA_STRUCTURES_H_
#define APF_DATA_STRUCTURES_H_

#include <vector>
#include <array>
#include <string>

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

struct Pose {
    double x;
    double y;
    double z;
    double yaw;
    std::string frame;
};
struct Input {
    std::vector<Obstacle> obstacles;
    Pose target;
    Pose robot_pose;
    float apf_k;
    //float apf_beta_k;
};
#endif // APF_DATA_STRUCTURES_H_
