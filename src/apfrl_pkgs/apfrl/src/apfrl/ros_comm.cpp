#include <algorithm>
#include <cmath>

#include "apfrl/ros_comm.h"
#include <chrono>
#include <thread>

using namespace std::placeholders;
using namespace std::chrono;
RosComm::RosComm(): Node("apfrl"),apfrl_input_(), is_new_target_(false), is_new_scan_(false), 
                                is_new_odom_(false), is_got_target_(false), is_new_obs_(false), is_new_k_(false), is_new_apf_target_(false),
                                is_apf_got_target_(false)

{
  
    this->declare_parameter("model_path","");
    // Which observation scaling the loaded weights were trained under. Default
    // "barn" keeps current behaviour; "legacy" restores the pre-BARN contract
    // for apfrl_model.pt and stackelberg_model.pt. trial.launch.py sets it per
    // planner, so a policy can never be run against the wrong scaling.
    this->declare_parameter("obs_profile","barn");
    this->declare_parameter("rate",3.0);
    this->get_parameter("model_path",model_path_);
    this->get_parameter("rate",rate);

    RCLCPP_INFO(this->get_logger(),"Model Path: %s",model_path_.c_str());
    
    std::string obs_profile_name;
    this->get_parameter("obs_profile",obs_profile_name);
    const ObsProfile obs_profile = ParseObsProfile(obs_profile_name);
    max_range_ = ApfRl::MaxRangeFor(obs_profile);
    RCLCPP_INFO(this->get_logger(),"Observation profile: %s (lidar clamp %.1f m)",
                obs_profile_name.c_str(), max_range_);

    apfrl_ = std::make_shared<ApfRl>(model_path_, obs_profile);

    rate_  = std::make_unique<rclcpp::Rate>(rate);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Async TF wait — see apf::checkTransform for rationale.
    transform_check_timer_ = this->create_wall_timer(
        300ms, std::bind(&RosComm::checkTransform, this));

    //target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1U, std::bind(&RosComm::TargetCallback, this,_1));
    // SensorDataQoS, not the default. /scan is published BEST_EFFORT, as sensor
    // streams are; a RELIABLE subscriber is incompatible and receives nothing.
    // rclcpp logs "offering incompatible QoS, no messages will be sent to it"
    // once and carries on, so the planner just sits there with an empty scan.
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(), std::bind(&RosComm::ScanCallback,this,_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",1U, std::bind(&RosComm::OdomCallback,this,_1));
    current_apf_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/apfrl/current_apf_goal",1U);

    apfrl_service_ = this->create_service<apfrl_interfaces::srv::SendCommand>("/apfrl/send_command",
                              std::bind(&RosComm::CommandCallback, this, std::placeholders::_1, std::placeholders::_2));
    navigatior_client_ = this->create_client<apfrl_interfaces::srv::SendCommand>("/navigatior/send_command");
    
    navigate_to_pose_server_ = rclcpp_action::create_server<apfrl_interfaces::action::NavigateToPose>(
    this,
    "/apfrl/navigate_to_pose",
    // 1) Goal callback: decide ACCEPT / REJECT
    [this](
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const apfrl_interfaces::action::NavigateToPose::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(
        this->get_logger(),
        "Received goal request target x: %.2f, y: %.2f",
        goal->target_pose.pose.position.x,
        goal->target_pose.pose.position.y);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // 2) Cancel callback: must return CancelResponse
    [this](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<apfrl_interfaces::action::NavigateToPose>> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received cancel request — setting stop flag");
        stop_requested_.store(true);
        return rclcpp_action::CancelResponse::ACCEPT;
    },
    // 3) Accepted callback: start execution (often in a new thread)
    [this](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<apfrl_interfaces::action::NavigateToPose>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted, stopping previous execution if any");
        stop_requested_.store(true);
        {
            std::lock_guard<std::mutex> lock(execution_mutex_);
            stop_requested_.store(false);
        }
        RCLCPP_INFO(this->get_logger(), "Starting new execution");
        std::thread([this, goal_handle]() {
            this->executeNavigateToPose(goal_handle);
        }).detach();
    }
    );

    #if XAVIER
    humans_sub_ = this->create_subscription<zed_interfaces::msg::ObjectsStamped>("/humans",1U, std::bind(&RosComm::HumansCallback,this,_1));
    #else
    humans_sub_ = this->create_subscription<apfrl_interfaces::msg::Obstacles>("/humans",1U, std::bind(&RosComm::HumansCallback,this,_1));
    #endif

    wp_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal",1U);
    apfk_pub_ = this->create_publisher<std_msgs::msg::Float32>("/apf_k",1U);
    nav_status_pub_ = this->create_publisher<apfrl_interfaces::msg::NavigationStatus>("/navigation_status", 10);
    prev_apfrl_input_ = {};

}

bool RosComm::GetData(SocialInput &input)
{
    //RCLCPP_INFO(this->get_logger(),"APFRL Get Data Start");
    std::lock_guard<std::mutex> lock(data_mutex_);
    input = {};
    if (is_new_scan_)
    {
        prev_apfrl_input_.scan = apfrl_input_.scan;
        input.scan = apfrl_input_.scan;
        is_new_scan_ = false;
    }
    else
    {
        input.scan = prev_apfrl_input_.scan;
    }

    if (is_new_odom_)
    {
        input.robot_pose = apfrl_input_.robot_pose;
        prev_apfrl_input_.robot_pose = apfrl_input_.robot_pose;
        is_new_odom_ = false;
    }
    else
    {
        input.robot_pose = prev_apfrl_input_.robot_pose;
    }

    if (is_new_target_)
    {    
        TransformPose(apfrl_input_.target, "map");
        prev_apfrl_input_.target = apfrl_input_.target;
        TransformPose(apfrl_input_.target, "base_link");
        input.target = apfrl_input_.target;
        is_new_target_ = false;
    }
    else
    {
        if(!is_got_target_)
        {
            apfrl_input_ = {};
            return false;
        }
        input.target = prev_apfrl_input_.target;
        TransformPose(input.target, "base_link");
    }

    if (is_new_humans_)
    {
        input.humans = apfrl_input_.humans;
        is_new_humans_ = false;
    }
    else
    {
        input.humans = prev_apfrl_input_.humans;
        for (auto &human: input.humans)
        {
            TransformPose(human,"base_link");
        }
    }

    apfrl_input_ = {};
    //RCLCPP_INFO(this->get_logger(),"APFRL Get Data End");
    return true;

}


void RosComm::TargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target_msg)
{
    apfrl_input_.target.x = target_msg->pose.position.x;
    apfrl_input_.target.y = target_msg->pose.position.y;
    apfrl_input_.target.z = target_msg->pose.position.z;

    tf2::Quaternion q(
        target_msg->pose.orientation.x,
        target_msg->pose.orientation.y,
        target_msg->pose.orientation.z,
        target_msg->pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    apfrl_input_.target.yaw = yaw;
    apfrl_input_.target.frame = target_msg->header.frame_id;
    is_new_target_ = true;
    is_got_target_ = true;

}

void RosComm::ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
 // RCLCPP_INFO(this->get_logger(),"Scan Callback");
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto &ranges = scan_msg->ranges;
    const std::size_t n = ranges.size();
    if (n == 0U) {
        return;                       // nothing to downsample; keep the last scan
    }
    if (!(scan_msg->angle_increment > 0.0F)) {
        RCLCPP_ERROR_ONCE(this->get_logger(),
                          "scan angle_increment %.6f is not positive; cannot place policy bins",
                          scan_msg->angle_increment);
        return;
    }
    apfrl_input_.scan.resize(STATE_SCAN_SIZE);
    // Bin k has to hold the ray the policy learned to find there. Every policy
    // here trained on the TurtleBot3 gpu_lidar, which sweeps 0 -> 2*pi from
    // base_scan's +x (turtlebot3_burger.gazebo.xacro, base_scan at rpy 0 0 0), so
    // its ranges[::16] put bin k at 2*pi*k/N counter-clockwise from FORWARD: bin 0
    // ahead, 16 left, 32 behind, 48 right. Measured, not assumed: that sensor in a
    // test world with a wall 1.9 m ahead reads 1.90 m at bin 0.
    //
    // semprob's scan sweeps -pi -> pi instead. Taking its rays in index order put
    // forward at bin 32 and handed every policy the world turned 180 degrees --
    // the wall behind the robot appeared ahead of it. So index by angle, from the
    // message's own angle_min/angle_increment, and any sweep lands bin k on the
    // same bearing. The scan must be in base_link (pointcloud_to_laserscan's
    // target_frame), since the bearing is taken from the robot's forward axis.
    //
    // Indexing stays bounded by n: the old loop walked a compile-time SCAN_SIZE
    // (3200, from a different lidar) unchecked, read ~2100 elements past this
    // robot's ~1025 rays, and was the source of the intermittent SIGSEGV.
    const double two_pi = 2.0 * M_PI;
    for (std::size_t k = 0U; k < STATE_SCAN_SIZE; ++k)
    {
        const double bearing = two_pi * static_cast<double>(k) / STATE_SCAN_SIZE;
        double offset = std::fmod(bearing - scan_msg->angle_min, two_pi);
        if (offset < 0.0) {
            offset += two_pi;
        }
        const auto idx = static_cast<std::size_t>(std::lround(offset / scan_msg->angle_increment));
        const auto range = ranges[std::min(idx, n - 1U)];
        apfrl_input_.scan[k] =
            (range > max_range_ || std::isnan(range)) ? max_range_ : range;
    }
    is_new_scan_ = true;
}


void RosComm::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    //    RCLCPP_INFO(this->get_logger(),"Odom Callback");
    std::lock_guard<std::mutex> lock(data_mutex_);
    tf2::Quaternion q(
    odom_msg->pose.pose.orientation.x,
    odom_msg->pose.pose.orientation.y,
    odom_msg->pose.pose.orientation.z,
    odom_msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    apfrl_input_.robot_pose.x = odom_msg->pose.pose.position.x;
    apfrl_input_.robot_pose.y = odom_msg->pose.pose.position.y;
    apfrl_input_.robot_pose.yaw = yaw;
    is_new_odom_ = true;
}

void RosComm::checkTransform()
{
    if (transform_ready_.load()) {
        return;
    }
    if (tf_buffer_->canTransform("base_link", "odom", tf2::TimePointZero))
    {
        try {
            odom_transform_ = tf_buffer_->lookupTransform("base_link", "odom", tf2::TimePointZero);
            transform_ready_.store(true);
            transform_check_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "TF base_link<->odom obtained, planner ready");
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup transient failure: %s", ex.what());
        }
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for base_link<->odom transform...");
    }
}

void RosComm::TransformPose(Pose& pose, std::string target_frame)
{
    try {
        odom_transform_ = tf_buffer_->lookupTransform(target_frame, pose.frame, tf2::TimePointZero);
        tf2::Vector3 current_position(pose.x, pose.y, pose.z);
        tf2::Stamped<tf2::Transform> transform;
        tf2::fromMsg(odom_transform_, transform);
        current_position = transform(current_position);

        pose.x = current_position.getX();
        pose.y = current_position.getY();
        pose.z = current_position.getZ();
        pose.frame = target_frame;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TransformPose failed: %s", ex.what());
    }
}

void RosComm::CommandCallback(
  const std::shared_ptr<apfrl_interfaces::srv::SendCommand::Request> request,
  std::shared_ptr<apfrl_interfaces::srv::SendCommand::Response> response)
{
    RCLCPP_INFO(this->get_logger(),"APFRL Command Callback: %d", request->command);
    switch (static_cast<Commands>(request->command))
    {
        case Commands::STOP:
        {
            RCLCPP_INFO(this->get_logger(), "Received STOP command, setting stop flag");
            stop_requested_.store(true);
            response->success = true;
            response->message = "APFRL Stop requested";
            break;
        }
        case Commands::RESET:
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            apfrl_input_ = {};
            prev_apfrl_input_ = {};
            is_new_target_ = false;
            is_new_scan_ = false;
            is_new_odom_ = false;
            is_got_target_ = false;
            is_new_humans_ = false;
            is_new_obs_ = false;
            is_new_apf_target_ = false;
            is_new_k_ = false;
            stop_requested_.store(false);
            response->success = true;
            response->message = "APFRL Reset Done";
            break;
        }
        case Commands::SET_TARGET:
        {
            RCLCPP_INFO(this->get_logger(), "Received SET_TARGET command");
            if (request->float_params.size() >= 3) {
                std::lock_guard<std::mutex> tl(target_mutex_);
                shared_map_target_.x = request->float_params[0];
                shared_map_target_.y = request->float_params[1];
                shared_map_target_.z = 0.0;
                shared_map_target_.yaw = request->float_params[2] * M_PI / 180.0;
                shared_map_target_.frame = "map";
                target_updated_.store(true);
                RCLCPP_INFO(this->get_logger(), "SET_TARGET: updated target to (%.2f, %.2f, %.1f deg)",
                            shared_map_target_.x, shared_map_target_.y, request->float_params[2]);
                response->success = true;
                response->message = "APFRL target updated";
            } else {
                RCLCPP_WARN(this->get_logger(), "SET_TARGET requires float_params[x, y, theta]");
                response->success = false;
                response->message = "SET_TARGET requires float_params[x, y, theta]";
            }
            break;
        }
        default:
        {
            response->success = false;
            response->message = "Unknown Command: " + std::to_string(request->command);
            break;
        }
    }
}

#if XAVIER
void RosComm::HumansCallback(const zed_interfaces::ObjectsStampedConstPtr &humans_msg) {
  apfrl_input_.humans = {};
  prev_apfrl_input_.humans = {};
  auto frame = humans_msg->header.frame_id;
  for (auto human : humans_msg->objects) {
    Pose temp_human {};
    temp_human.x = human.position[0];
    temp_human.y = human.position[1];
    temp_human.frame = frame;
    prev_apfrl_input_.humans.push_back(temp_human);
    TransformPose(temp_human,"base_link");

    apfrl_input_.humans.push_back(temp_human);
  }
  is_new_humans_ = true;
}
#else
void RosComm::HumansCallback(const apfrl_interfaces::msg::Obstacles::SharedPtr humans_msg)
{
    apfrl_input_.humans = {};
    prev_apfrl_input_.humans = {};
    auto frame = humans_msg->header.frame_id;
    for (auto human : humans_msg->obs)
    {
        Pose temp_human {};
        temp_human.x = human.pose.x;
        temp_human.y = human.pose.y;

        temp_human.frame = frame;
        prev_apfrl_input_.humans.push_back(temp_human);
        TransformPose(temp_human,"base_link");
        auto dist = sqrt(temp_human.x*temp_human.x + temp_human.y*temp_human.y);
        if (dist > kMaxHumanRange)
        {
            continue;
        }
        apfrl_input_.humans.push_back(temp_human);
    }
    is_new_humans_ = true;
}
#endif


// The modular half of the contract: the policy chooses a waypoint and a gain,
// apf_node turns them into velocity. This is what every training launch wired
// up (policy -> /goal + /apf_k -> apf_node -> /cmd_vel), and what social_apfrl
// already does. The waypoint is in base_link because that is the frame the
// policy emits it in; apf_node's TargetCallback transforms from the header.
//
// A stop is expressed as a waypoint at the robot's own origin: apf.yaml's
// goal_distance_threshold (0.01 m) then puts apf_node in its at-goal hold and
// it commands zero, so there is no second authority publishing velocity.
void RosComm::PublishWaypoint(double wp_x, double wp_y, double apf_k)
{
    geometry_msgs::msg::PoseStamped wp_msg{};
    wp_msg.header.stamp = this->now();
    wp_msg.header.frame_id = "base_link";
    wp_msg.pose.position.x = wp_x;
    wp_msg.pose.position.y = wp_y;
    wp_msg.pose.orientation.w = 1.0;
    wp_pub_->publish(wp_msg);

    std_msgs::msg::Float32 k_msg{};
    k_msg.data = static_cast<float>(apf_k);
    apfk_pub_->publish(k_msg);
}

void RosComm::PublishNavigationStatus(int status, double distance_to_target, const std::string& message)
{
    auto status_msg = apfrl_interfaces::msg::NavigationStatus();
    status_msg.header.stamp = this->now();
    status_msg.source_node = apfrl_interfaces::msg::NavigationStatus::SOURCE_APFRL;
    status_msg.status = status;
    status_msg.distance_to_target = distance_to_target;
    status_msg.message = message;
    nav_status_pub_->publish(status_msg);
}

void RosComm::executeNavigateToPose(std::shared_ptr<rclcpp_action::ServerGoalHandle<apfrl_interfaces::action::NavigateToPose>> goal_handle)
{
    std::lock_guard<std::mutex> lock(execution_mutex_);
    static rclcpp::Time prev_time_apfrl = this->now();

    RCLCPP_INFO(this->get_logger(), "Executing NavigateToPose action");

    auto result = std::make_shared<apfrl_interfaces::action::NavigateToPose::Result>();
    auto feedback = std::make_shared<apfrl_interfaces::action::NavigateToPose::Feedback>();

    // Brief wait for TF if a goal arrives during startup; abort if it never appears.
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!transform_ready_.load() && std::chrono::steady_clock::now() < deadline) {
            if (!rclcpp::ok() || stop_requested_.load()) break;
            std::this_thread::sleep_for(100ms);
        }
        if (!transform_ready_.load()) {
            RCLCPP_WARN(this->get_logger(), "Aborting goal: TF base_link<->odom never appeared");
            goal_handle->abort(result);
            return;
        }
    }

    const auto goal = goal_handle->get_goal();

    Pose target_pose;
    target_pose.x = goal->target_pose.pose.position.x;
    target_pose.y = goal->target_pose.pose.position.y;
    target_pose.z = goal->target_pose.pose.position.z;

    tf2::Quaternion q(
        goal->target_pose.pose.orientation.x,
        goal->target_pose.pose.orientation.y,
        goal->target_pose.pose.orientation.z,
        goal->target_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    target_pose.yaw = yaw;
    target_pose.frame = goal->target_pose.header.frame_id;
    
    apfrl_input_.target = target_pose;
    is_new_target_=true;
    is_got_target_ = true;
    stop_requested_.store(false);

    // Keep a map-frame copy for distance calculation
    Pose map_target = target_pose;
    
    RCLCPP_INFO(this->get_logger(), "New Target Recieved in frame %s", target_pose.frame.c_str());
    
    TransformPose(target_pose, "base_link");

    double distance_to_target = std::hypot(target_pose.x, target_pose.y);
    while (rclcpp::ok() && distance_to_target > 0.1)
    {
        if (stop_requested_.load())
        {
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            // Stop by asking apf_node to hold: a waypoint at the robot's own
            // origin is inside apf.yaml's goal_distance_threshold, so its
            // at-goal branch commands zero. No second velocity authority.
            PublishWaypoint(0.0, 0.0, 0.0);
            if (goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
            }
            else
            {
                goal_handle->abort(result);
            }
            is_new_target_ = false;
            is_got_target_ = false;
            is_new_apf_target_ = false;
            is_new_k_ = false;
            is_apf_got_target_ = false;
            stop_requested_.store(false);
            return;
        }

        // Check for runtime target update (from navigator SET_TARGET command)
        if (target_updated_.load())
        {
            std::lock_guard<std::mutex> tl(target_mutex_);
            map_target = shared_map_target_;
            apfrl_input_.target = shared_map_target_;
            is_new_target_ = true;
            is_got_target_ = true;
            target_updated_.store(false);
            RCLCPP_INFO(this->get_logger(), "Target updated during execution to (%.2f, %.2f) frame=%s",
                        map_target.x, map_target.y, map_target.frame.c_str());
        }

        // Transform target to base_link for accurate distance
        Pose target_bl = map_target;
        TransformPose(target_bl, "base_link");
        distance_to_target = std::hypot(target_bl.x, target_bl.y);

        feedback->distance_remaining = distance_to_target;
        goal_handle->publish_feedback(feedback);

        // Simulate navigation logic (replace with actual logic)
        SocialInput apfrl_input{};
        Output output{};

        // One policy step per 1/rate seconds of SIM time -- 3 Hz, the step
        // every policy trained at. The loop below polls every 10 ms of wall
        // time so the gate opens within a few ms of the boundary; it used to
        // sleep 100 ms, so after ~0.33 s the gate only opened on the next
        // ~130 ms iteration and the policy stepped at ~2-2.5 Hz (measured on a
        // recorded trial), holding each waypoint ~1.5x as long as in training.
        if (GetData(apfrl_input) &&
            (this->now() - prev_time_apfrl) >= rclcpp::Duration::from_seconds(1.0 / rate))
        {
            //RCLCPP_INFO(this->get_logger(),"APFRL GET DATA");
            apfrl_->Step(apfrl_input, output);


            geometry_msgs::msg::PointStamped current_goal_msg{};
            current_goal_msg.header.stamp = this->now();
            current_goal_msg.header.frame_id = "base_link";
            current_goal_msg.point.x = output.wp_x;
            current_goal_msg.point.y = output.wp_y;
            current_apf_goal_pub_->publish(current_goal_msg);

            // The node's whole output: apf_node turns these into velocity.
            PublishWaypoint(output.wp_x, output.wp_y, output.apf_k);

            is_new_apf_target_ = true;
            is_new_k_ = true;
            is_apf_got_target_ = true;

            prev_time_apfrl = this->now();

        }

        // Recalculate distance to target. Short on purpose: this is the polling
        // resolution of the policy-step gate above, not the policy rate.
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    // Check cancellation one more time before completing
    if (stop_requested_.load())
    {
        RCLCPP_INFO(this->get_logger(), "Goal canceled at completion");
        // See above: hold via a waypoint at the origin rather than a Twist.
        PublishWaypoint(0.0, 0.0, 0.0);
        if (goal_handle->is_canceling())
        {
            goal_handle->canceled(result);
        }
        else
        {
            goal_handle->abort(result);
        }
        stop_requested_.store(false);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal reached");
        result->success = true;
        goal_handle->succeed(result);
    }

    is_new_target_ = false;
    is_got_target_ = false;
    is_new_apf_target_ = false;
    is_new_k_ = false;
    is_apf_got_target_ = false;
}
