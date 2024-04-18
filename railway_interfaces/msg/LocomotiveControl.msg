uint8 SET_SPEED = 1
uint8 SET_DIRECTION = 2
uint8 SET_FUNCTION = 3

uint8 DIRECTION_NO_CHANGE = 0
uint8 DIRECTION_FORWARD = 1
uint8 DIRECTION_REVERSE = 2
uint8 DIRECTION_TOGGLE = 3

uint8 FUNCTION_STATE_FALSE = 0
uint8 FUNCTION_STATE_TRUE = 1

uint8 PROTOCOL_UNKNOWN = 0
uint8 PROTOCOL_ROS = 1
uint8 PROTOCOL_MM1 = 2
uint8 PROTOCOL_MM2 = 3
uint8 PROTOCOL_DCC = 4
uint8 PROTOCOL_MFX = 5

uint8 command
uint8 track_index  # Not used, for future implementation
uint8 protocol
int32 address
int16 speed
uint8 direction
uint8 function_index
uint8 function_state
uint8 maintenance_speed_step
