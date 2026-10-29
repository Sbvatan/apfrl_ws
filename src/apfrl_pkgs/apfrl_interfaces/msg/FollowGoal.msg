# A single follow-goal: where the body should drive to (relative to a tracked
# actor) and where that actor currently is. Emitted by actors_pub from sim
# ground-truth poses.
int32 actor_no                   # Numeric tag parsed from "actor<N>"
geometry_msgs/Pose2D follow_pose # Goal pose for the body (m, m, rad)
geometry_msgs/Pose2D actor_pose  # Current actor pose (m, m, rad)
# Current actor velocity. Present in the ROS1 message (apfrl_msgs/FollowGoal)
# and dropped during the port, while coord_env still reads g.actor_velocity -
# so the coordination env raised AttributeError and died the moment a goal
# actually reached it. It never had, because actors_pub could not see actors at
# all, which is why the missing field stayed hidden.
geometry_msgs/Twist actor_velocity
