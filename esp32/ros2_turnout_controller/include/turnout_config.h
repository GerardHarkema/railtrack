// !!! This is an automated generated header file, do not modify by your self !!!
// Timestamp: 15/02/2024 14:41:28
#ifndef _TURNOUT_CONFIG_
#define _TURNOUT_CONFIG_

// Agent config generated from: ../../config/micro_ros_agent_config.json
#define SSID   "BirdsBoven"
#define PASSWORD   "Highway12!"
uint8_t ip_address[4] = {192, 168, 2, 150};
#define PORT   8888

// Turnout config generated from: /home/gerard/modelspoor_ws/src/config/turnout_config_a.json
#define  NODE_NAME  "turnout_group_a"

#define  STATUS_LED  23

TURNOUT_CONFIG turnout_config[] = {
		{MAGNET, 11, {.magnet = {33, 32}}},
		{MAGNET, 11, {.magnet = {26, 25}}},
		{MAGNET, 12, {.magnet = {14, 27}}},
		};
#define  NUMBER_OF_TURNOUTS  3

#endif //_TURNOUT_CONFIG_
