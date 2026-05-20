std_msgs/Header header
float32 voltage
float32 current
float32 percent
bool is_charging
bool low_battery_warning    # below 20%
bool critical_battery       # below 10%, triggers RTH
