
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#if XAVIER
#include <zed_interfaces/msg/objects_stamped.hpp>
#endif
#include <iostream>


using namespace std::chrono;

bool is_new = false;
geometry_msgs::msg::PoseStamped goal {};

class Human_Goal_Node : public rclcpp::Node
{
  public:
  rclcpp::Rate rate;
  Human_Goal_Node() : Node("moving_goal_pub"), rate(5.0)
  {
    // nh.getParam("/goal_pub/goal_x", goal_x);
    // nh.getParam("/goal_pub/goal_y", goal_y);
    // nh.getParam("/goal_pub/goal_vx", goal_vx);
    // nh.getParam("/goal_pub/goal_vy", goal_vy);

  goal_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal",1);
#if XAVIER
    human_sub = this->create_subscription<zed_interfaces::msg::ObjectsStamped>("/zed_node/obj_det/objects",1U, std::bind(&Human_Goal_Node::HumanCallback,this,std::placeholders::_1));
#endif
  }
  
  void Publish()
  {   
      
      goal.header.frame_id = "base_link";
      goal.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
      goal.pose.orientation.w = 1;      
      goal_pub->publish(goal);
      rate.sleep();
  }
  private:
  
  double goal_x, goal_y, goal_vx, goal_vy;
#if XAVIER
  rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr human_sub;
#endif
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;

#if XAVIER
  void HumanCallback(const zed_interfaces::msg::ObjectsStamped::SharedPtr humans_msg) 
  {
    if (!humans_msg->objects.empty()) 
    {
      auto human = humans_msg->objects[0];
      auto dx = human.position[0];
      auto dy = human.position[1];
      auto dist = std::sqrt(dx*dx + dy*dy);
      auto scale = dist > 1.5 ? (dist-1.5)/dist : 0.0;
      goal.pose.position.x = scale * dx;
      goal.pose.position.y = scale * dy;
      is_new = true;
      std::cout << "is new " << goal.pose.position.x << std::endl;
    }
  }

#endif
};

int main(int argc, char **argv) {
  rclcpp::init(argc,argv);
  //rclcpp::NodeHandle nh;
  

  std::shared_ptr<Human_Goal_Node> node = std::make_shared<Human_Goal_Node>();

  while (rclcpp::ok()) {
    if (is_new){
      node->Publish();
      is_new = false;
      rclcpp::spin_some(node);
    } 

  }
  return 0;
}

