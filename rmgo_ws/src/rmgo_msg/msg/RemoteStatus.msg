std_msgs/Header header

uint8 POWER_LIMIT_UNKNOWN=0
uint8 POWER_LIMIT_NORMAL=1
uint8 POWER_LIMIT_BURST=2
uint8 POWER_LIMIT_CHARGE=3

uint8 SHOOT_FREQUENCY_UNKNOWN=0
uint8 SHOOT_FREQUENCY_LOW=1
uint8 SHOOT_FREQUENCY_HIGH=2
uint8 SHOOT_FREQUENCY_BURST=3

uint8 TARGET_UNKNOWN=0
uint8 TARGET_ARMOR=1
uint8 TARGET_SMALL_BUFF=2
uint8 TARGET_BIG_BUFF=3

uint8 ARMOR_UNKNOWN=0
uint8 ARMOR_ALL=1
uint8 ARMOR_OUTPOST_BASE=2

bool active
bool fire_pressed
bool cover_open
bool gimbal_eject
uint8 power_limit_state
uint8 shoot_frequency
uint8 target
uint8 armor_target
bool target_color_red
