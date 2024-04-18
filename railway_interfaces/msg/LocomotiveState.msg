uint8 DIRECTION_FORWARD = 1
uint8 DIRECTION_REVERSE = 2

uint8 FUNCTION_STATE_FALSE = 0
uint8 FUNCTION_STATE_TRUE = 1

uint8 track_index  # Not used, for future implementation
uint8 on_track_segment # Not used, for future implementation
uint8 protocol
uint32 address
uint16 speed
uint8 direction
bool[] function_state

