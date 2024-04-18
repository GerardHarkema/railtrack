uint8 CONTROLLER_CAN             = 0
uint8 CONTROLLER_DCC             = 1
uint8 CONTROLLER_MM              = 2
uint8 CONTROLLER_DCC_MM          = 3

uint8 track_index  # Not used, for future implementation
bool state
float32 current
float32 voltage
float32 temperature
bool current_overload
bool voltage_overload
bool temperature_overload
uint8 controller_type
uint8 operating_mode

