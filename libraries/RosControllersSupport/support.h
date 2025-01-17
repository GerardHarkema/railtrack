#ifndef RAILTRACK_SUPPORT
#define RAILTRACK_SUPPORT

#include "ros_messages.h"
#include "defines.h"

void lookupLocomotiveProtocol(uint8_t protocol, char *protocol_txt);
void error_loop();
char* getDirectionTxt(int direction);
bool lookupTurnoutIndex(int turnout_number, int *turnout_index);
bool lookupLocomotiveIndex(int locomotive_address, uint8_t protocol, int *locomotive_index);

#endif //RAILTRACK_SUPPORT