// !!! This is an automated generated header file, do not modify by your self !!!
// Timestamp: 15/01/2025 20:07:29
#ifndef _TRACK_CONFIG_
#define _TRACK_CONFIG_

// Track config generated from: ../../config/track_config_DCC.json
LOCOMOTIVE active_locomotives[] = {{60, railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC, SS_128}, {3, railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC, SS_128}, {30, railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2, SS_NOT_USED}, {78, railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2, SS_NOT_USED}, {50, railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1, SS_NOT_USED}, };
#define  NUMBER_OF_ACTIVE_LOCOMOTIVES   5

unsigned short int active_turnouts_mm[] = {1, 2, 3, 4, };
#define  NUMBER_OF_ACTIVE_TURNOUTS_MM   4

unsigned short int active_turnouts_ros[] = {11, 12, 21, 22, 23, 24, };
#define  NUMBER_OF_ACTIVE_TURNOUTS_ROS   6

#endif //_TRACK_CONFIG_
