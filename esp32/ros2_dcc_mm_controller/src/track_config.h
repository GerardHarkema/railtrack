#ifndef TRACK_CONFIG
#define TRACK_CONFIG

#include "micro_ros_includes.h"
#include "defines.h"
#include "tft_printf.h"
#include <TrackPacket.h>
#include <TrackPacketQueueList.h>
#include <TrackPacketScheduler.h>

void track_config_callback(const void * msgin);

#endif