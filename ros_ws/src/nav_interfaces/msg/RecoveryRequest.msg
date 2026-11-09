std_msgs/Header header
string request_id
uint8 state
int8 local_side
geometry_msgs/Pose robot_pose
string reason

uint8 BLOCKED=0
uint8 ESCAPING=1
uint8 CLEARED=2
uint8 STILL_BLOCKED=3
uint8 RECOVERED=4
uint8 ABORTED=5
