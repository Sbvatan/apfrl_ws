#ifndef APFRL_INTERFACES_PARAMETERS_H
#define APFRL_INTERFACES_PARAMETERS_H

//#define SONAROB_FACTORY
//#define SONAROB_BU
//#define SONAROB_EMIRGAN
#define SONAROB_SIM

enum LidarTypes {
    NoLidar       	= 0,
    Slamtech_T1M4 	= 1,
    YdLidar_Tmini_plus 	= 2
};


enum CameraTypes {
    NoCamera     = 0,
    Zed2         = 1,
    Oak_D_Pro    = 2
};

#if defined(SONAROB_FACTORY)
static int  LIDAR_TYPE   =  LidarTypes::Slamtech_T1M4;
static int CAMERA_TYPE   =  CameraTypes::Zed2;
#elif defined(SONAROB_BU)
static int  LIDAR_TYPE   =  LidarTypes::Slamtech_T1M4;
static int CAMERA_TYPE   =  CameraTypes::Zed2;
#elif defined(SONAROB_EMIRGAN)
static int  LIDAR_TYPE   =  LidarTypes::YdLidar_Tmini_plus;
static int CAMERA_TYPE   =  CameraTypes::Oak_D_Pro;
#elif defined(SONAROB_SIM)
static int  LIDAR_TYPE   =  LidarTypes::NoLidar;
static int CAMERA_TYPE   =  CameraTypes::NoCamera;
#endif

#include <string>
#include <list>

#define NODE_CORE               "core_controller"
#define NODE_HUBMOTOR           "can_controller_hubmotors"
#define NODE_PTU                "servo_ptu"
#define NODE_MANUAL_NAV         "manual_nav"
#define NODE_NAVIGATOR          "navigator"
#define NODE_ZED                "zed_wrapper"           //  "zed_wrapper"
#define NODE_OAK                "depthai_ros_driver"    //  "depthai_ros_driver"
#define NODE_SLAMTECH           "sllidar_ros2"
#define NODE_YDLIDAR            "ydlidar_ros_driver"
#define NODE_LASER_FUSION       "laser_fusion"
#define NODE_ULTRASONIC         "bagrob_ultrasonic"
#define NODE_HUMAN_DETECT       "human_detect_cpp"
#define NODE_ODOM_HANDLER       "odom_handler"
#define NODE_LOADCELL           "loadcell"
#define NODE_APF                "apf"
#define NODE_APFRL              "apfrl"
#define NODE_HEAD_APF           "head_apf"
#define NODE_OBSTACLE_GENERATOR "obs_generator_laser"
#define NODE_OBSTACLE_MERGE     "obstacle_merge_node"
#define NODE_OBSTACLE_GENERATOR_CONTAINER "obstacle_generator_container"
#define NODE_SLAM_TOOLBOX       "slam_toolbox"
#define NODE_EKF_FILTER_1        "bagrob_ekf_filter_node_1"
#define NODE_EKF_FILTER_2        "bagrob_ekf_filter_node_2"
#define NODE_TRANSFORM_LISTENER  "transform_listener"


const std::list<std::string> NODE_LIST  { NODE_CORE,
                                        NODE_HUBMOTOR,
                                        NODE_PTU,
                                        NODE_ZED,
                                        NODE_OAK,
                                        NODE_SLAMTECH,
                                        NODE_YDLIDAR,
                                        NODE_LASER_FUSION,
                                        NODE_ULTRASONIC,
                                        NODE_LOADCELL,
                                        NODE_NAVIGATOR,
                                        NODE_MANUAL_NAV,
                                        NODE_ODOM_HANDLER,
                                        NODE_OBSTACLE_GENERATOR,
                                        NODE_OBSTACLE_MERGE,
                                        NODE_TRANSFORM_LISTENER,
                                        NODE_SLAM_TOOLBOX,
                                        NODE_EKF_FILTER_1,
                                        NODE_EKF_FILTER_2,
                                        NODE_APF,
                                        NODE_APFRL,
                                        NODE_HEAD_APF,
                                        NODE_HUMAN_DETECT,
                                        };
// HUBMOTOR PARAMETERS
    constexpr static double wheel_distance = 0.48284; // Distance between wheels
    constexpr static double wheel_radius = 0.1165; // Radius of the wheels (was 0.1265)
    constexpr int encoder_scale = 4096 ; // 4096 counts per revolution
    constexpr double hubmotor_speed_resolution = 10; // RPM sent to motor gets multiplied by 1/10 for better resolution

#endif
