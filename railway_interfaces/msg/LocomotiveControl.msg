int8 SET_SPEED = 1
int8 SET_DIRECTION = 2
int8 SET_FUNCTION = 3

int8 DIRECTION_NO_CHANGE = 0
int8 DIRECTION_FORWARD = 1
int8 DIRECTION_REVERSE = 2
int8 DIRECTION_TOGGLE = 3

int8 FUNCTION_STATE_FALSE = 0
int8 FUNCTION_STATE_TRUE = 1

int8 command
int32 address
int16 speed
# direction: false = forward true = reverse
int8 direction
int8 function_index
int8 function_state
