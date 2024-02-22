// !!! This is an automated generated header file, do not modify by your self !!!
// Timestamp: 16/02/2024 20:58:24
#ifndef _TRACK_CONFIG_
#define _TRACK_CONFIG_

// Agent config generated from: ../../config/micro_ros_agent_config.json
#define SSID   "BirdsBoven"
#define PASSWORD   "Highway12!"
uint8_t ip_address[4] = {192, 168, 2, 150};
#define PORT   8888

// Track config generated from: ../../config/track_config.json
LOCOMOTIVE active_locomotives[] = {{5, MFX, 0},{60, DCC, 0},{6, MFX, 0},{78, MM1,  0}};
#define  NUMBER_OF_ACTIVE_LOCOMOTIVES   4

unsigned short int active_turnouts_c[] = {1, 2, 3, 4};
#define  NUMBER_OF_ACTIVE_TURNOUTS_C   4

unsigned short int active_turnouts_m[] = {11, 12, 21, 22, 23, 24};
#define  NUMBER_OF_ACTIVE_TURNOUTS_M   6

#endif //_TRACK_CONFIG_
