#include "apf/ros_comm.h"
#include <chrono>
#include <thread>

using namespace std::placeholders;
RosComm::RosComm(double rate) : Node("apf"), apf_data_({}), apf_prev_data_({}), rate_(rate),
                                is_new_target_(false), is_new_obs_(false), is_got_target_(false),
                                is_new_odom_(false), apf_only_k_coef_(1.5), minGamma_(0.0),
                                goal_reach_threshold_(0.1), control_period_(0.1)
{

    this->declare_parameter("xMax", 1.0);
    this->get_parameter("xMax", xScale);

    // Declare and read APF parameters from config
    this->declare_parameter("robot_radius", 0.4);
    this->declare_parameter("max_linear_velocity", 0.2);
    this->declare_parameter("min_linear_velocity", -0.1);
    this->declare_parameter("max_angular_velocity", M_PI / 8.0);
    this->declare_parameter("min_gamma", 0.0);
    this->declare_parameter("goal_distance_threshold", 0.01);
    this->declare_parameter("neighborhood_radius", 3.0);
    this->declare_parameter("apf_k_coefficient", 1.5);
    this->declare_parameter("goal_reach_threshold", 0.1);
    this->declare_parameter("control_period", 0.1);
    this->declare_parameter("verbose", false);
    // Which gradPhi formulation to steer with. Default is the ORIGINAL analytic
    // form, so this node with no parameters behaves exactly as the pre-2026-09
    // apf_ext did; the stable form is opt-in, per the workspace rule that new
    // behaviour is default-off.
    //
    // The two describe the same Phi, but they do not agree in general. The
    // analytic form raises Gamma to the apf_k-th power and subtracts two such
    // terms, and loses its steering angle once apf_k passes ~0.35 x the NEARBY
    // obstacle count -- tightest when obstacles are few. That is exactly a BARN
    // room: 4-8 obstacles within reach, so the two forms diverge even at
    // apf_k 5. The earlier claim that they "agree to ~1e-15 rad over the range
    // used" held for the rooms it was measured in and not for these.
    //
    // So the form must match what the weights trained against, and nothing
    // else: the policies exported 2026-08-28 (apfrl, stackelberg, social_apfrl)
    // trained on analytic -- it was the only form -- and the BARN pair trained
    // on stable, set explicitly in both of their launches. Running a policy on
    // the other form is a train/deploy mismatch: measured, apfrl on stable in
    // BARN drove at 0.28 m/s against its 0.51 m/s in the house and never came
    // within 3 m of a goal in 171 rooms.
    this->declare_parameter("use_stable_gradient", false);
    // Bounds on apf_k, applied to whatever source supplied it. The upper bound
    // was a hardcoded 5.0, which silently defeated the count-based heuristic
    // right below (1.5 * obstacle count reaches 66 at the ~44 obstacles the
    // laser generator produces here) and equally capped whatever the RL policy
    // asked for. Kept at 5.0 by default so behaviour is unchanged.
    this->declare_parameter("apf_k_min", 1.0);
    // Default 5.0, the original hardcoded ceiling, and it must move TOGETHER
    // with the gradient default above. Analytic at a ceiling of 20 is the one
    // combination that is genuinely broken: plain apf's heuristic (1.5 x ~40
    // obstacles) saturates at 20, far past where the analytic angle holds, and
    // the robot wedges. Measured on small_house: four trials that succeeded in
    // August at a ceiling of 5 all timed out at 20, ~95 m driven against
    // ~91-94 m of odom drift. Any launch relying on defaults -- including
    // semprob_robot/navigation.launch.py, the real robot -- would have got it.
    //
    // The ceiling costs the RL policies nothing: they map their action to
    // apf_k as (action + 1.5) * 2, which is [1, 5] by construction, in training
    // and at deploy alike. Raise it only together with use_stable_gradient:=true.
    this->declare_parameter("apf_k_max", 5.0);

    this->get_parameter("verbose", verbose_);

    double robot_radius, max_v, min_v, max_w, min_gamma, d_th_goal, neighborhood_radius;
    this->get_parameter("robot_radius", robot_radius);
    this->get_parameter("max_linear_velocity", max_v);
    this->get_parameter("min_linear_velocity", min_v);
    this->get_parameter("max_angular_velocity", max_w);
    this->get_parameter("min_gamma", min_gamma);
    this->get_parameter("goal_distance_threshold", d_th_goal);
    this->get_parameter("neighborhood_radius", neighborhood_radius);
    this->get_parameter("apf_k_coefficient", apf_only_k_coef_);
    this->get_parameter("goal_reach_threshold", goal_reach_threshold_);
    this->get_parameter("control_period", control_period_);
    this->get_parameter("apf_k_min", apf_k_min_);
    this->get_parameter("apf_k_max", apf_k_max_);

    apf_.setParams(robot_radius, max_v, min_v, max_w, min_gamma, d_th_goal, neighborhood_radius);
    const bool use_stable = this->get_parameter("use_stable_gradient").as_bool();
    apf_.setUseStableGradient(use_stable);
    RCLCPP_INFO(this->get_logger(), "gradPhi formulation: %s",
                use_stable ? "stable (reformulated)" : "analytic (original)");

    RCLCPP_INFO(this->get_logger(), "APF params: robot_r=%.2f, max_v=%.2f, min_v=%.2f, max_w=%.3f, "
                                    "min_gamma=%.3f, d_th_goal=%.3f, neighborhood_r=%.1f, k_coef=%.2f, "
                                    "goal_reach=%.2f, ctrl_period=%.2f, k_range=[%.1f,%.1f]",
                robot_radius, max_v, min_v, max_w, min_gamma, d_th_goal, neighborhood_radius,
                apf_only_k_coef_, goal_reach_threshold_, control_period_,
                apf_k_min_, apf_k_max_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Async TF wait — the executor isn't spinning yet, so a synchronous wait
    // here would deadlock. Timer fires only after the node is spinning.
    transform_check_timer_ = this->create_wall_timer(
        300ms, std::bind(&RosComm::checkTransform, this));

    // Obstacles come from obs_generator, which publishes /obstacles.
    // This used to read /merged_obstacles, the output of a node that fused laser
    // with ultrasonic; that node was removed as obsolete and nothing publishes
    // that topic any more, so the planner silently received no obstacles at all.
    // Declared as a parameter so a future merged source can be selected without
    // another code change.
    const std::string obstacles_topic =
        this->declare_parameter<std::string>("obstacles_topic", "/obstacles");
    obs_sub_ = this->create_subscription<apfrl_interfaces::msg::Obstacles>(
        obstacles_topic, 1U, std::bind(&RosComm::ObsCallback, this, _1));
    apfk_sub_ = this->create_subscription<std_msgs::msg::Float32>("/apf_k", 1U, std::bind(&RosComm::ApfKCallback, this, _1));
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/apf/enable", 1U,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { enabled_.store(msg->data); });
    // minBetaCurrent_pub_ = this->create_publisher<std_msgs::msg::Float32>("/minBetaCurrent", 1U);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 1U, std::bind(&RosComm::OdomCallback, this, _1));
    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal", 1U, std::bind(&RosComm::TargetCallback, this, _1));
    // ROS time, not wall time: under use_sim_time this follows /clock, so the
    // APF updates every control_period of SIM time in every world. A wall timer
    // tied the control rate to how fast the simulator happened to run -- ~26
    // updates per sim second in the house (RTF 0.38) against ~10 in BARN
    // (RTF 1), and fewer still in training, which steps gz faster than real
    // time. Without sim time (the real robot) ROS time is system time, so
    // nothing changes there.
    control_timer_ = this->create_timer(
        std::chrono::duration<double>(control_period_),
        std::bind(&RosComm::ControlTimerCallback, this));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/apf/cmd_vel", 1U);
    nav_status_pub_ = this->create_publisher<apfrl_interfaces::msg::NavigationStatus>("/apf/navigation_status", 10);

    navigatior_client_ = this->create_client<apfrl_interfaces::srv::SendCommand>("/navigator/send_command");

    apf_service_ = this->create_service<apfrl_interfaces::srv::SendCommand>("/apf/send_command",
                                                                             std::bind(&RosComm::CommandCallback, this, std::placeholders::_1, std::placeholders::_2));
    navigate_to_pose_server_ = rclcpp_action::create_server<apfrl_interfaces::action::NavigateToPose>(
        this,
        "/apf/navigate_to_pose",
        // 1) Goal callback: decide ACCEPT / REJECT
        [this](
            const rclcpp_action::GoalUUID &uuid,
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
            // Signal any running execution thread to stop, then wait for it to release the mutex
            stop_requested_.store(true);
            {
                std::lock_guard<std::mutex> lock(execution_mutex_);
                // Previous thread has finished; clear stop flag for the new goal
                stop_requested_.store(false);
            }
            RCLCPP_INFO(this->get_logger(), "Starting new execution");
            std::thread([this, goal_handle]()
                        { this->executeNavigateToPose(goal_handle); })
                .detach();
        });

    // apf_betak_sub_ = nh_.subscribe<std_msgs::Float32>("/apf_beta_k",1U, &RosComm::ApfBetaKCallback,this);
    apf_data_ = {};
    apf_prev_data_ = {};
}

void RosComm::ObsCallback(const apfrl_interfaces::msg::Obstacles::SharedPtr obs_msg)
{
    apf_data_.obstacles.clear();

    for (auto &obs : obs_msg->obs)
    {
        Obstacle temp_obs{};
        temp_obs.x = obs.pose.x;
        temp_obs.y = obs.pose.y;
        temp_obs.yaw = obs.pose.theta;
        temp_obs.r_a = obs.r_a;
        temp_obs.r_b = obs.r_b;
        apf_data_.obstacles.push_back(temp_obs);
    }
    // RCLCPP_INFO(this->get_logger(),"Lidar Obstacle Received");
    is_new_obs_ = true;
}

void RosComm::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    apf_data_.robot_pose.x = odom_msg->pose.pose.position.x;
    apf_data_.robot_pose.y = odom_msg->pose.pose.position.y;
    apf_data_.robot_pose.yaw = yaw;
    is_new_odom_ = true;
}

void RosComm::ApfKCallback(const std_msgs::msg::Float32::SharedPtr k_msg)
{
    apf_data_.apf_k = k_msg->data;
    // is_new_k_ = true;
}
// void RosComm::ApfBetaKCallback(const std_msgs::Float32ConstPtr &betak_msg) {
//   data_.apf_beta_k = betak_msg->data;
// }

void RosComm::ControlTimerCallback()
{
    if (!transform_ready_.load())
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "[CTRL] waiting for TF base_link<->odom");
        return;
    }
    if (!enabled_.load())
    {
        // Held false by the trainer across a reset. Publish nothing at all, so
        // the only writer on cmd_vel is whoever is braking the robot.
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[CTRL] disabled");
        return;
    }
    if (!is_got_target_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[CTRL] no target yet");
        return;
    }
    if (!execution_mutex_.try_lock())
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[CTRL] action server active, skipping");
        return;
    }
    execution_mutex_.unlock();

    Input apf_input{};
    Velocity velocity{};
    bool got_data = GetData(apf_input);
    if (verbose_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[CTRL] got_data=%d  target=(%.2f,%.2f) frame=%s  obs=%zu  k=%.2f  odom=(%.2f,%.2f)",
                             got_data,
                             apf_input.target.x, apf_input.target.y, apf_input.target.frame.c_str(),
                             apf_input.obstacles.size(), apf_input.apf_k,
                             apf_input.robot_pose.x, apf_input.robot_pose.y);
    }

    if (got_data)
    {
        apf_.calculateVel(apf_input, velocity);
        if (verbose_)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[CTRL] vel: linear=%.3f  angular=%.3f", velocity.linear, velocity.angular);
        }
        geometry_msgs::msg::TwistStamped out_msg{};
        out_msg.header.stamp = this->get_clock()->now();
        out_msg.twist.linear.x = velocity.linear;
        out_msg.twist.angular.z = velocity.angular;
        vel_pub_->publish(out_msg);
    }
}

void RosComm::TargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target_msg)
{
    apf_data_.target.x = target_msg->pose.position.x;
    apf_data_.target.y = target_msg->pose.position.y;
    apf_data_.target.z = target_msg->pose.position.z;
    tf2::Quaternion q(
        target_msg->pose.orientation.x,
        target_msg->pose.orientation.y,
        target_msg->pose.orientation.z,
        target_msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    apf_data_.target.yaw = yaw;
    apf_data_.target.frame = target_msg->header.frame_id;
    is_new_target_ = true;
    is_got_target_ = true;
}

bool RosComm::GetData(Input &input)
{
    input = {};

    if (is_new_obs_)
    {
        input.obstacles = apf_data_.obstacles;
        apf_prev_data_.obstacles = apf_data_.obstacles;
        is_new_obs_ = false;
    }
    else
    {
        input.obstacles = apf_prev_data_.obstacles;
    }

    if (is_new_odom_)
    {
        input.robot_pose = apf_data_.robot_pose;
        apf_prev_data_.robot_pose = apf_data_.robot_pose;
        is_new_odom_ = false;
    }
    else
    {
        input.robot_pose = apf_prev_data_.robot_pose;
    }

    if (is_new_target_)
    {
        try
        {
            // To keep it in odom frame, if given in base_link frame
            TransformPose(apf_data_.target, "odom");
            apf_prev_data_.target = apf_data_.target;
            TransformPose(apf_data_.target, "base_link");
            input.target = apf_data_.target;
            is_new_target_ = false;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "[GetData] target transform failed: %s", ex.what());
            apf_data_ = {};
            return false;
        }
    }
    else
    {
        if (!is_got_target_)
        {
            apf_data_ = {};
            return false;
        }
        try
        {
            input.target = apf_prev_data_.target;
            TransformPose(input.target, "base_link");
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "[GetData] prev-target transform failed: %s", ex.what());
            apf_data_ = {};
            return false;
        }
    }

    // Prefer RL-provided k from /apf_k. If not available yet, reuse the last
    // valid k; only then fall back to the deterministic obstacle-count heuristic.
    float selected_apf_k = apf_prev_data_.apf_k;
    if (apf_data_.apf_k > 0.0F)
    {
        selected_apf_k = apf_data_.apf_k;
    }
    else if (selected_apf_k <= 0.0F)
    {
        selected_apf_k = static_cast<float>(apf_only_k_coef_ * input.obstacles.size());
    }

    selected_apf_k = std::max(static_cast<float>(apf_k_min_),
                              std::min(static_cast<float>(apf_k_max_), selected_apf_k));
    input.apf_k = selected_apf_k;
    apf_prev_data_.apf_k = selected_apf_k;

    apf_data_ = {};
    // transform target to base_link here (for both old or new)
    return true;
}
void RosComm::PublishVelocity(const Velocity &velocity)
{
    geometry_msgs::msg::TwistStamped out_msg{};
    out_msg.header.stamp = this->get_clock()->now();
    out_msg.twist.linear.x = velocity.linear;
    out_msg.twist.angular.z = velocity.angular;
    vel_pub_->publish(out_msg);
    rate_.sleep();
}

// void RosComm::getMinBetaCurrentFromAPF(double minBetaCurrent)
// {
//     minBetaCurrent_ = minBetaCurrent;
// }

// void RosComm::PublishMinBetaCurrent()
// {
//     std_msgs::msg::Float32 minBetaMsg{};
//     minBetaMsg.data = (float)minBetaCurrent_;
//     minBetaCurrent_pub_->publish(minBetaMsg);
// }

void RosComm::PublishNavigationStatus(int status, double distance_to_target, const std::string &message)
{
    apfrl_interfaces::msg::NavigationStatus status_msg;
    status_msg.header.stamp = this->now();
    status_msg.header.frame_id = "odom";
    status_msg.source_node = apfrl_interfaces::msg::NavigationStatus::SOURCE_APF;
    status_msg.status = status;
    status_msg.distance_to_target = static_cast<float>(distance_to_target);
    status_msg.message = message;
    nav_status_pub_->publish(status_msg);
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

void RosComm::TransformPose(Pose &pose, std::string target_frame)
{
    odom_transform_ = tf_buffer_->lookupTransform(target_frame, pose.frame, tf2::TimePointZero);
    tf2::Vector3 current_position(pose.x, pose.y, pose.z);
    tf2::Stamped<tf2::Transform> transform;
    tf2::fromMsg(odom_transform_, transform);

    current_position = transform(current_position);
    pose.x = current_position.getX();
    pose.y = current_position.getY();
    pose.z = current_position.getZ();
    pose.frame = target_frame;
}

void RosComm::CommandCallback(
    const std::shared_ptr<apfrl_interfaces::srv::SendCommand::Request> request,
    std::shared_ptr<apfrl_interfaces::srv::SendCommand::Response> response)
{
    int cmd = request->command;
    // STOP = 2, RESET = 3 (Commands enum)
    switch (static_cast<Commands>(cmd))
    {
    case Commands::STOP:
        RCLCPP_INFO(this->get_logger(), "Received STOP command, setting stop flag");
        stop_requested_.store(true);
        break;
    case Commands::RESET:
        RCLCPP_INFO(this->get_logger(), "Received RESET command, resetting APF state");
        // For simplicity, we can just set a flag or clear data to reset APF
        apf_data_ = {};
        apf_prev_data_ = {};
        is_new_target_ = false;
        is_got_target_ = false;
        stop_requested_.store(false);
        break;
    case Commands::SET_TARGET:
    {
        RCLCPP_INFO(this->get_logger(), "Received SET_TARGET command");
        if (request->float_params.size() >= 3)
        {
            std::lock_guard<std::mutex> tl(target_mutex_);
            shared_map_target_.x = request->float_params[0];
            shared_map_target_.y = request->float_params[1];
            shared_map_target_.z = 0.0;
            shared_map_target_.yaw = request->float_params[2] * M_PI / 180.0;
            shared_map_target_.frame = "map";
            target_updated_.store(true);
            RCLCPP_INFO(this->get_logger(), "SET_TARGET: updated target to (%.2f, %.2f, %.1f deg)",
                        shared_map_target_.x, shared_map_target_.y, request->float_params[2]);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "SET_TARGET requires float_params[x, y, theta]");
        }
        break;
    }
    default:
        RCLCPP_WARN(this->get_logger(), "Received unknown command: %d", cmd);
    }
    response->success = true;
    response->message = "APF command executed: " + std::to_string(request->command);
}

void RosComm::executeNavigateToPose(std::shared_ptr<rclcpp_action::ServerGoalHandle<apfrl_interfaces::action::NavigateToPose>> goal_handle)
{
    std::lock_guard<std::mutex> lock(execution_mutex_);
    rclcpp::Time prev_time_apf = this->now();

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
            result->success = false;
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

    apf_data_.target = target_pose;
    is_new_target_ = true;
    is_got_target_ = true;
    stop_requested_.store(false);

    // Keep a map-frame copy for distance calculation (GetData zeroes apf_data_)
    Pose map_target = target_pose;

    RCLCPP_INFO(this->get_logger(), "New Target Recieved in frame %s", target_pose.frame.c_str());

    TransformPose(target_pose, "base_link");

    double distance_to_target = std::hypot(target_pose.x, target_pose.y);
    while (rclcpp::ok() && distance_to_target > goal_reach_threshold_)
    {
        if (stop_requested_.load())
        {
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            // Publish zero velocity to stop the robot
            Velocity zero_vel{};
            zero_vel.linear = 0.0;
            zero_vel.angular = 0.0;
            PublishVelocity(zero_vel);
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
            stop_requested_.store(false);
            return;
        }

        // Check for runtime target update (from navigator SET_TARGET command)
        if (target_updated_.load())
        {
            std::lock_guard<std::mutex> tl(target_mutex_);
            map_target = shared_map_target_;
            apf_data_.target = shared_map_target_;
            is_new_target_ = true;
            is_got_target_ = true;
            target_updated_.store(false);
            RCLCPP_INFO(this->get_logger(), "Target updated during execution to (%.2f, %.2f) frame=%s",
                        map_target.x, map_target.y, map_target.frame.c_str());
        }

        // Transform target to base_link each iteration for accurate distance
        // (robot is always at origin in base_link frame)
        Pose target_bl = map_target;
        TransformPose(target_bl, "base_link");
        distance_to_target = std::hypot(target_bl.x, target_bl.y);

        RCLCPP_INFO(this->get_logger(), "TARGET (map): (%.2f, %.2f), DISTANCE: %.2f",
                    map_target.x, map_target.y, distance_to_target);

        feedback->distance_remaining = distance_to_target;
        goal_handle->publish_feedback(feedback);

        Input apf_input{};
        Velocity velocity{};
        if ((this->now() - prev_time_apf) >= rclcpp::Duration::from_seconds(control_period_))
        {
            if (GetData(apf_input))
            {
                apf_.calculateVel(apf_input, velocity);
                PublishVelocity(velocity);

                prev_time_apf = this->now();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "APF input data not ready, skipping this cycle");
            }
        }
        // Recalculate distance to target
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // Check cancellation one more time before completing
    if (stop_requested_.load())
    {
        RCLCPP_INFO(this->get_logger(), "Goal canceled at completion");
        Velocity zero_vel{};
        zero_vel.linear = 0.0;
        zero_vel.angular = 0.0;
        PublishVelocity(zero_vel);
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
    return;
}
