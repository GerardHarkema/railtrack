#ifndef TRACK_LOCOMOTIVE
#define TRACK_LOCOMOTIVE

#include <micro_ros_includes.h>
#include <defines.h>
#include "tft_printf.h"


void locomotive_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void locomotive_control_callback(const void * msgin);
bool getProtocolFromAddress(uint address, uint *subaddress, uint8_t *protocol);
#endif // TRACK_LOCOMOTIVE