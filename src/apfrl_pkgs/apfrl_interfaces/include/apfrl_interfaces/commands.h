#ifndef APFRL_INTERFACES_COMMANDS_H
#define APFRL_INTERFACES_COMMANDS_H

#pragma once

#include <iostream>

enum class OdomType : uint8_t
{
    ODOM_OPENLOOP = 1,
    ODOM_ENCODER = 2,
    ODOM_RGBD = 3,
    ODOM_ENCODER_RGBD = 4
};

inline std::string to_string(OdomType o) {
    switch (o) {
        case OdomType::ODOM_OPENLOOP: return "ODOM_OPEN_LOOP";
        case OdomType::ODOM_ENCODER: return "ODOM_ENCODER";
        case OdomType::ODOM_RGBD: return "ODOM_RGBD";
        case OdomType::ODOM_ENCODER_RGBD: return "ODOM_ENCODER+RGBD";
        default: return "UNKNOWN";
    }
}

enum class MotionType : uint8_t
{
    UNDEFINED = 0,
    SMOOTH = 1,
    SEQUENTIAL = 2,
    CONSTANT_STRAIGHT = 3,
    CONSTANT_ARC = 4,
    CONTINUOUS = 5
};

inline std::string to_string(MotionType m) {
    switch (m) {
        case MotionType::UNDEFINED: return "UNDEFINED";
        case MotionType::SMOOTH: return "SMOOTH";
        case MotionType::SEQUENTIAL: return "SEQUENTIAL";
        case MotionType::CONSTANT_STRAIGHT: return "CONSTANT_STRAIGHT";
        case MotionType::CONSTANT_ARC: return "CONSTANT_ARC";
        case MotionType::CONTINUOUS: return "CONTINUOUS";
        default: return "UNDEFINED";
    }
}

enum class Commands : uint8_t
{
    INITIALIZE = 0,
    SHUTDOWN = 1,
    SET_HOME = 2,
    SET_SLEEP = 3,
    SET_POSITION = 4,
    SET_VELOCITY = 5,
    SET_MODE = 6,
    SET_ODOMETRY = 7,
    SET_STATE = 8,
    SET_GAZE = 9,
    SET_EXPRESSION = 10,
    SET_ACCELERATION = 11,
    SET_DECELERATION = 12,
    SET_TARGET = 13,
    TARGET_REACHED = 20,
    STOP = 21,
    TOGGLE_SENSOR = 22,
    RESET = 23,
    DEMO_START = 30,
    DEMO_STOP = 31,
    RESET_ODOM = 40,
    REGISTER_TARGET = 41,
    // can_hubmotors related
    GET_BATTERY = 50,
    GET_CURRENT_POSITION = 51,
    GET_CURRENT_SPEED = 52,
    GET_TEMPERATURE = 53
};

enum class ServoCommands : uint8_t
{
    SET_GAZE = 1,
    SET_MODE = 2,
    SET_HOME = 3
};

enum class ServoMode : uint8_t
{
    IDLE = 0,
    TRACK_HUMAN = 1,
    SCAN = 2,
    FIXED_POSITION = 3
};

enum class CoreCommands : uint8_t
{
    SET_NAVIGATION_MODE = 1,
    SET_NAVIGATION_ODOMETRY = 2,
    SET_FEEDBACK = 3,
    SET_TARGET = 4,
    RESET_SYSTEM = 10,
    EMERGENCY_STOP = 11,
    CLEAR_EMERGENCY_STOP = 12,
    START_MISSION = 20,
    CANCEL_MISSION = 21,
    PAUSE_MISSION = 22,
    RESUME_MISSION = 23,
    GET_STATUS = 30,
    BATTERY_STATUS = 31,
    GET_NODE_STATUS = 32,
    REGISTER_HUMAN_SUCCESS = 40,
    REGISTER_HUMAN_FAILURE = 41,
    HUMAN_LOST = 42,
    HUMAN_FOUND = 43,
    OBS_DEADLOCK_DETECTED = 50,
    LOCALIZATION_DRIFT_DETECTED = 51,
    MANUAL_CONTROL_START = 60,
    MANUAL_CONTROL_STOP = 61,
    HUMAN_FOLLOW_ACCOMPLISHED = 70,
    GO_HOME_ACCOMPLISHED = 71,
    HUMAN_FOLLOW_WAITING = 80,
    HUMAN_FOLLOW_RESUMING = 81,
    STOP_ALL_MOTION = 90,
    SET_SLAM_MAPPING_MODE = 100,
    SAVE_SLAM_MAPPING = 101,
    SET_SLAM_LOCALIZATION_MODE = 102,
    LOAD_SLAM_MAPPING = 103,
    NAVIGATION_FAILED = 110,
};

inline std::string to_string(CoreCommands c)
{
    switch (c)
    {
    case CoreCommands::SET_NAVIGATION_MODE:
        return "SET_NAVIGATION_MODE";
        break;
    case CoreCommands::SET_NAVIGATION_ODOMETRY:
        return "SET_NAVIGATION_ODOMETRY";
        break;
    case CoreCommands::SET_FEEDBACK:
        return "SET_FEEDBACK";
        break;
    case CoreCommands::SET_TARGET:
        return "SET_TARGET";
        break;
    case CoreCommands::RESET_SYSTEM:
        return "RESET_SYSTEM";
        break;
    case CoreCommands::EMERGENCY_STOP:
        return "EMERGENCY_STOP";
        break;
    case CoreCommands::CLEAR_EMERGENCY_STOP:
        return "CLEAR_EMERGENCY_STOP";
        break;
    case CoreCommands::START_MISSION:
        return "START_MISSION";
        break;
    case CoreCommands::CANCEL_MISSION:
        return "CANCEL_MISSION";
        break;
    case CoreCommands::PAUSE_MISSION:
        return "PAUSE_MISSION";
        break;
    case CoreCommands::RESUME_MISSION:
        return "RESUME_MISSION";
        break;
    case CoreCommands::GET_STATUS:
        return "GET_STATUS";
        break;
    case CoreCommands::BATTERY_STATUS:
        return "BATTERY_STATUS";
        break;
    case CoreCommands::GET_NODE_STATUS:
        return "GET_NODE_STATUS";
        break;
    case CoreCommands::REGISTER_HUMAN_SUCCESS:
        return "REGISTER_HUMAN_SUCCESS";
        break;
    case CoreCommands::REGISTER_HUMAN_FAILURE:
        return "REGISTER_HUMAN_FAILURE";
        break;
    case CoreCommands::HUMAN_LOST:
        return "HUMAN_LOST";
        break;
    case CoreCommands::HUMAN_FOUND:
        return "HUMAN_FOUND";
        break;
    case CoreCommands::OBS_DEADLOCK_DETECTED:
        return "OBS_DEADLOCK_DETECTED";
        break;
    case CoreCommands::LOCALIZATION_DRIFT_DETECTED:
        return "LOCALIZATION_DRIFT_DETECTED";
        break;
    case CoreCommands::MANUAL_CONTROL_START:
        return "MANUAL_CONTROL_START";
        break;
    case CoreCommands::MANUAL_CONTROL_STOP:
        return "MANUAL_CONTROL_STOP";
        break;
    case CoreCommands::HUMAN_FOLLOW_ACCOMPLISHED:
        return "HUMAN_FOLLOW_ACCOMPLISHED";
        break;
    case CoreCommands::GO_HOME_ACCOMPLISHED:
        return "GO_HOME_ACCOMPLISHED";
        break;
    case CoreCommands::HUMAN_FOLLOW_WAITING:
        return "HUMAN_FOLLOW_WAITING";
        break;
    case CoreCommands::HUMAN_FOLLOW_RESUMING:
        return "HUMAN_FOLLOW_RESUMING";
        break;
    case CoreCommands::STOP_ALL_MOTION:
        return "STOP_ALL_MOTION";
        break;
    case CoreCommands::SET_SLAM_MAPPING_MODE:
        return "SET_SLAM_MAPPING_MODE";
        break;
    case CoreCommands::SAVE_SLAM_MAPPING:
        return "SAVE_SLAM_MAPPING";
        break;
    case CoreCommands::SET_SLAM_LOCALIZATION_MODE:
        return "SET_SLAM_LOCALIZATION_MODE";
        break;
    case CoreCommands::LOAD_SLAM_MAPPING:
        return "LOAD_SLAM_MAPPING";
        break;
    case CoreCommands::NAVIGATION_FAILED:
        return "NAVIGATION_FAILED";
        break;
    default:
        return "UNKNOWN_COMMAND";
        break;
    }
}

enum class TopicNames : uint8_t
{
    NavigationManual = 1,  // /manual_nav/cmd_vel (direct control from manual_nav node)
    NavigationAuto   = 2   // /cmd_vel (from APF/APFRL/DWA navigation)
};

enum class TopicAction : uint8_t
{
    TopicUnsubscribe = 0,
    TopicSubscribe = 1
};

inline std::string to_string(TopicAction a) {
    switch (a) {
        case TopicAction::TopicUnsubscribe: return "Unsubscribe";
        case TopicAction::TopicSubscribe:   return "Subscribe";
        default: return "TopicUnknown";
    }
}

enum class NodeNames : uint8_t
{
    NodeCoreController = 0,
    NodeCanHubmotor = 1,
    NodeManualNav = 2,
    NodeAPF = 3,
    NodeAPFRL = 4,
    NodeBagrobRobotLocalization = 5,
    NodeHeadApf = 6,
    NodeServoPtu = 7,
    NodeHumanDetectCPP = 8,
    NodeLoadcell = 9,
    NodeObsGeneratorRPLidar = 10,
    NodeOdomHandler = 11,
    NodeLaserFusion = 12,
    NodeSLLidar = 13,
    NodeGuiMotion = 20
};

enum class NavigatorCommands
{
    SET_MODE = 1,           // Set navigation mode (int_params[0] = mode)
    SET_TARGET = 2,         // Set navigation target (float_params = [x, y, theta])
    SET_ODOMETRY = 3,       // Set odometry type
    STOP = 4,               // Stop current navigation
    RESET = 5,              // Reset all navigation nodes
    GET_STATUS = 6,         // Get current status
    SET_GOAL_MODE = 7,      // Set goal mode (int_params[0] = 0:MANUAL, 1:AUTO)
    GENERATE_NEW_GOAL = 8,  // Generate and navigate to a new auto goal
    START_AUTO_GOAL = 9,    // Start automatic goal navigation
    TARGET_REACHED = 10,    // When the corresponding motion node reaches the target
};

enum class NavigationMode {
    NAVIGATION_IDLE = 0,
    NAVIGATION_MANUAL = 1,
    NAVIGATION_APF = 2,
    NAVIGATION_APFRL = 3,
    NAVIGATION_DWA = 4,
    NAVIGATION_AUTO_APF = 5,
    NAVIGATION_AUTO_APFRL = 6,
    NAVIGATION_AUTO_DWA = 7,
};

inline std::string to_string(NavigationMode mode)
{
    switch (mode)
    {
        case NavigationMode::NAVIGATION_IDLE: return "IDLE";
        case NavigationMode::NAVIGATION_MANUAL: return "MANUAL_NAV";
        case NavigationMode::NAVIGATION_APF: return "APF_ONLY";
        case NavigationMode::NAVIGATION_APFRL: return "APFRL";
        case NavigationMode::NAVIGATION_DWA: return "DWA";
        case NavigationMode::NAVIGATION_AUTO_APF: return "AUTO_APF";
        case NavigationMode::NAVIGATION_AUTO_APFRL: return "AUTO_APFRL";
        case NavigationMode::NAVIGATION_AUTO_DWA: return "AUTO_DWA";
        default: return "UNKNOWN";
    }
}

// Should be in this order to match the order of sensors in the robot
// and to match the order of the sensors in the robot localization filter
// Odometry, Pose, Twist, Imu
enum Sensors
{
    Encoder,
    Visual_odometry,
    Ble,
    Imu,
};

enum class CoreState
{
    BOOT,
    IDLE,
    AUTONOMOUS,
    MANUAL_OPERATION,
    FAULT,
    PAUSED,
    RECOVERY,
    CHARGING,
    EMERGENCY_STOP,
    TELEOP,
    DOCKING,
};

inline std::string to_string(CoreState s)
{
    switch (s)
    {
    case CoreState::BOOT:
        return "BOOT";
    case CoreState::IDLE:
        return "IDLE";
    case CoreState::AUTONOMOUS:
        return "AUTONOMOUS";
    case CoreState::PAUSED:
        return "PAUSED";
    case CoreState::RECOVERY:
        return "RECOVERY";
    case CoreState::TELEOP:
        return "TELEOP";
    case CoreState::DOCKING:
        return "DOCKING";
    case CoreState::CHARGING:
        return "CHARGING";
    case CoreState::EMERGENCY_STOP:
        return "EMERGENCY_STOP";
    case CoreState::FAULT:
        return "FAULT";
    case CoreState::MANUAL_OPERATION:
        return "MANUAL_OPERATION";
    }
    return "UNKNOWN";
}

/**
 * @brief Navigation status
 */
enum class NavigationStatus
{
    IDLE = 0,
    NAVIGATING = 1,
    TARGET_REACHED = 2,
    BLOCKED = 3,
    ERROR = 4
};

inline std::string to_string(NavigationStatus status)
{
    switch (status)
    {
        case NavigationStatus::IDLE: return "IDLE";
        case NavigationStatus::NAVIGATING: return "NAVIGATING";
        case NavigationStatus::TARGET_REACHED: return "TARGET_REACHED";
        case NavigationStatus::BLOCKED: return "BLOCKED";
        case NavigationStatus::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

enum MissionType
{
    NONE,
    ROBOT_FOLLOWING_HUMAN,
    HUMAN_FOLLOWING_ROBOT,
    RECOVER_LOST_HUMAN,
    GO_HOME,
    AUTO_GOAL_NAVIGATION
};

inline std::string to_string(MissionType m)
{
    switch (m)
    {
    case MissionType::NONE:
        return "NONE";
    case MissionType::AUTO_GOAL_NAVIGATION:
        return "AUTO_GOAL_NAVIGATION";
    case MissionType::GO_HOME:
        return "GO_HOME";
    case MissionType::HUMAN_FOLLOWING_ROBOT:
        return "HUMAN_FOLLOWING_ROBOT";
    case MissionType::RECOVER_LOST_HUMAN:
        return "RECOVER_LOST_HUMAN";
    case MissionType::ROBOT_FOLLOWING_HUMAN:
        return "ROBOT_FOLLOWING_HUMAN";
    }
    return "UNKNOWN";
}

#endif // APFRL_INTERFACES_COMMANDS_H
