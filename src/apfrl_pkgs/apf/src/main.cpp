#include "apf/ros_comm.h"
#include "apf/data_structures.h"
#include "apf/apf_ext.hpp"
#include "apfrl_interfaces/msg/navigation_status.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto ros_comm = std::make_shared<RosComm>(20.0);
    rclcpp::spin(ros_comm);
    rclcpp::shutdown();
    return 0;
}
