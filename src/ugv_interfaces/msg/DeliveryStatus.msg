std_msgs/Header header

uint8 IDLE = 0
uint8 NAVIGATING_TO_PICKUP = 1
uint8 WAITING_AT_PICKUP = 2
uint8 NAVIGATING_TO_DROPOFF = 3
uint8 WAITING_AT_DROPOFF = 4
uint8 RETURNING_HOME = 5
uint8 DOCKING = 6
uint8 EMERGENCY_STOP = 7
uint8 FAULT = 8

uint8 state
string mission_id
string current_waypoint_id
uint32 waypoints_remaining
float32 estimated_distance_m
float32 battery_percent
bool teleop_override_active
string status_message
