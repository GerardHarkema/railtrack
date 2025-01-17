#ifndef TRACK_CONFIG
#define TRACK_CONFIG

#include "micro_ros_includes.h"
#include "defines.h"
#include "tft_printf.h"
#include <TrackPacket.h>

bool enoughNeededEeprom(int number_of_locomotives, int number_of_turnouts);
void dumpConfiguration();
void track_config_callback(const void * msgin);
void init_turnouts();
void init_locomotives();

#endif