#ifndef TRACK_TURNOUT
#define TRACK_TURNOUT

#include "micro_ros_includes.h"
#include "defines.h"
#include "tft_printf.h"


void turnout_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void turnout_control_callback(const void * msgin);
#endif // TRACK_TURNOUT