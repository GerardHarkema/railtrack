#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H


typedef struct{
  char* ssid;
  char* password;
  IPAddress server_ip_address;
}NETWORK_CONFIG;

bool configureNetwork(bool forceConfigure, NETWORK_CONFIG *networkConfig);

#endif