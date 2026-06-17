std_msgs/Header header

uint8 MODE_DISABLED=0
uint8 MODE_FRICTION=1
uint8 MODE_SINGLE=2
uint8 MODE_AUTOMATIC=3
uint8 MODE_EJECT=4
uint8 MODE_DEEP_EJECT=5

uint8 mode
bool friction_requested
bool friction_ready
bool friction_faulted
bool bullet_fired
float64 left_friction_velocity
float64 right_friction_velocity
float64 left_control_velocity
float64 right_control_velocity
