#ifndef TRACK_LOCOMOTIVE
#define TRACK_LOCOMOTIVE

#include "micro_ros_includes.h"
#include "defines.h"
#include "tft_printf.h"
#include <DCCPacket.h>
#include "DCCPacketQueueList.h"
#include <DCCPacketScheduler.h>

void locomotive_state_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void locomotive_control_callback(const void * msgin);
#endif // TRACK_LOCOMOTIVE