#ifndef TRACK_POWER
#define TRACK_POWER

#include "micro_ros_includes.h"
#include "defines.h"
#include "tft_printf.h"
#include <DCCPacket.h>
#include "DCCPacketQueueList.h"
#include <DCCPacketScheduler.h>

void power_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void power_control_callback(const void * msgin);

#endif // TRACK_POWER