#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "data_structures.h"
#include "apfrl/ros_comm.h"
#include "apfrl/apfrl.h"
#include "apfrl_interfaces/msg/navigation_status.hpp"
#include <string>
#include <cmath>

// Helper function to convert NavState to NavigationStatus status
int navStateToStatus(NavState state) {
    switch(state) {
        case NavState::WAIT:
            return apfrl_interfaces::msg::NavigationStatus::STATUS_IDLE;
        case NavState::RESUME:
            return apfrl_interfaces::msg::NavigationStatus::STATUS_NAVIGATING;
        case NavState::SUCCESS:
            return apfrl_interfaces::msg::NavigationStatus::STATUS_TARGET_REACHED;
        case NavState::COLLISION:
            return apfrl_interfaces::msg::NavigationStatus::STATUS_BLOCKED;
        case NavState::ALMOST:
            return apfrl_interfaces::msg::NavigationStatus::STATUS_NAVIGATING;
        default:
            return apfrl_interfaces::msg::NavigationStatus::STATUS_IDLE;
    }
}

// Helper function to get status message
std::string getStatusMessage(NavState state) {
    switch(state) {
        case NavState::WAIT:
            return "Waiting for target";
        case NavState::RESUME:
            return "Navigating to target";
        case NavState::SUCCESS:
            return "Target reached";
        case NavState::COLLISION:
            return "Collision detected";
        case NavState::ALMOST:
            return "Almost at target";
        default:
            return "Unknown state";
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Inference loop is driven by the action server inside RosComm
    // (see executeNavigateToPose in ros_comm.cpp); main only spins the executor.
    auto ros_comm = std::make_shared<RosComm>();
    rclcpp::spin(ros_comm);
    rclcpp::shutdown();
    return 0;
}
