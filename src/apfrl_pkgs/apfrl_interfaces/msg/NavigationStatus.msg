# Navigation status message for feedback from navigation nodes to navigator

std_msgs/Header header

# Source node identifier
# 1 = APF, 2 = APFRL, 3 = MANUAL_NAV, 4 = DWA
int32 source_node

# Navigation status
# 0 = IDLE, 1 = NAVIGATING, 2 = TARGET_REACHED, 3 = BLOCKED, 4 = ERROR
int32 status

# Distance to target (meters)
float32 distance_to_target

# Additional message/description
string message

# Constants for source_node
int32 SOURCE_APF = 1
int32 SOURCE_APFRL = 2
int32 SOURCE_MANUAL_NAV = 3
int32 SOURCE_DWA = 4

# Constants for status
int32 STATUS_IDLE = 0
int32 STATUS_NAVIGATING = 1
int32 STATUS_TARGET_REACHED = 2
int32 STATUS_BLOCKED = 3
int32 STATUS_ERROR = 4
