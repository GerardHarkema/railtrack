// !!! This is an automated generated header file, do not modify by your self !!!
// Timestamp: 15/01/2025 20:56:07
#ifndef _TRACK_CONFIG_
#define _TRACK_CONFIG_

// Track config generated from: ../../config/track_config_DCC.json
LOCOMOTIVE active_locomotives[] = {{60, railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC, railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_128}, {3, railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC, railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_128}, {30, railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2, railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_UNKNOWN}, {78, railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2, railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_UNKNOWN}, {50, railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1, railway_interfaces__msg__LocomotiveControl__DCC_SPEEDSTEP_UNKNOWN}, };
#define  NUMBER_OF_ACTIVE_LOCOMOTIVES   5

unsigned short int active_turnouts_mm[] = {1, 2, 3, 4, };
#define  NUMBER_OF_ACTIVE_TURNOUTS_MM   4

unsigned short int active_turnouts_ros[] = {11, 12, 21, 22, 23, 24, };
#define  NUMBER_OF_ACTIVE_TURNOUTS_ROS   6

#endif //_TRACK_CONFIG_
