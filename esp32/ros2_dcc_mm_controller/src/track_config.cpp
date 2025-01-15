#include <Arduino.h>
#include <EEPROM.h>
#include "support.h"
#include "track_config.h"

#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif

extern bool track_config_enable_flag;
extern uint16_t *p_eeprom_programmed;
extern uint16_t *p_number_of_active_locomotives;
extern uint16_t *p_number_of_active_turnouts;
extern TRACK_OBJECT *p_locomtives;
extern TRACK_OBJECT *p_turnouts;
extern bool *p_turnout_status;

void dumpConfiguration(){
    DEBUG_PRINT("p_eeprom_programmed            = 0x%08x\n", p_eeprom_programmed);
    DEBUG_PRINT("p_number_of_active_locomotives = 0x%08x\n", p_number_of_active_locomotives);
    DEBUG_PRINT("p_number_of_active_turnouts    = 0x%08x\n", p_number_of_active_turnouts);
    DEBUG_PRINT("p_locomtives                   = 0x%08x\n", p_locomtives);
    DEBUG_PRINT("p_turnouts                     = 0x%08x\n", p_turnouts);
    DEBUG_PRINT("p_turnout_status               = 0x%08x\n", p_turnout_status);
    DEBUG_PRINT("eeprom_programmed 0x%04x\n", *p_eeprom_programmed);
    DEBUG_PRINT("number_of_locomotives %i\n", *p_number_of_active_locomotives);
    DEBUG_PRINT("number_of_turnouts %i\n", *p_number_of_active_turnouts);

    if(*p_eeprom_programmed == EEPROM_PROGRAMMED_TAG){
      DEBUG_PRINT("!!!\n");
      for(int i = 0; i < *p_number_of_active_locomotives; i++){
        DEBUG_PRINT("Locomitive %i, address = %i, protocol = %i\n", i+1, p_locomtives[i].address, p_locomtives[i].protocol);
      }
      for(int i = 0; i < *p_number_of_active_turnouts; i++){
        DEBUG_PRINT("Turnout %i, address = %i, protocol = %i, state = %i\n", i+1, p_turnouts[i].address, p_turnouts[i].protocol, p_turnout_status[i]);
      }
    }
    else
      DEBUG_PRINT("EEPROM not programmed\n");
}

bool enoughNeededEeprom(int number_of_locomotives, int number_of_turnouts){
  int needed_eeprom_size;

  needed_eeprom_size += sizeof(uint16_t); // eeprom programmed
  needed_eeprom_size += sizeof(uint16_t); // number_of_turnouts
  needed_eeprom_size += sizeof(uint16_t); // number_of_locomotive
  needed_eeprom_size += sizeof(TRACK_OBJECT) * number_of_locomotives; // needed_eeprom_size of locomotive objects
  needed_eeprom_size += sizeof(TRACK_OBJECT) * number_of_turnouts; // needed_eeprom_size of turnout objects
  needed_eeprom_size += sizeof(bool) * number_of_turnouts; // needed_eeprom_size of turnout_status
  DEBUG_PRINT("needed_eeprom_size = %i\n", needed_eeprom_size);
  if(needed_eeprom_size > EEPROM_SIZE){
    DEBUG_PRINT("needed_eeprom_size to large, size = %i, max = %i\n", needed_eeprom_size, EEPROM_SIZE);
  }
  return (needed_eeprom_size > EEPROM_SIZE) ? false: true;
}

void track_config_callback(const void * msgin){

  const railway_interfaces__msg__TrackConfig * track_config = (const railway_interfaces__msg__TrackConfig *)msgin;
  int number_of_turnouts = 0;
  int number_of_locomotives = 0;

  DEBUG_PRINT("Configuration\n received\n");


  if(track_config_enable_flag){
  // Check if the message contains track objects
    if (!track_config->track_objects.size) {
        *p_number_of_active_locomotives = 0;
        *p_number_of_active_turnouts = 0;
        memset(p_eeprom_programmed, 0x00, EEPROM_SIZE);
        EEPROM.commit();
        
        DEBUG_PRINT("Track-config factory reset\n");
        return;
    }

    // Count the turnouts & locomotives
    for (size_t i = 0; i < track_config->track_objects.size; ++i) {
        const railway_interfaces__msg__TrackObjectConfig * track_object = 
            &track_config->track_objects.data[i];
      switch(track_object->config_type){
        case railway_interfaces__msg__TrackObjectConfig__CONFIG_TYPE_LOCOMOTIVE:
          number_of_locomotives++;
          break;
        case railway_interfaces__msg__TrackObjectConfig__CONFIG_TYPE_TURNOUT:
          switch(track_object->protocol){
            case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1:
            case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2:
            case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC:
            case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MFX:
              number_of_turnouts++;
              break;
          }
          break;
      }
    }
    if(enoughNeededEeprom(number_of_locomotives, number_of_locomotives)){
      TRACK_OBJECT *p_iterator = p_locomtives;
      *p_number_of_active_locomotives = 0;
      *p_number_of_active_turnouts = 0;

      // Count the turnouts & locomotives
      for (size_t i = 0; i < track_config->track_objects.size; ++i) {
          const railway_interfaces__msg__TrackObjectConfig * track_object = 
        &track_config->track_objects.data[i];
#if 1
        switch(track_object->config_type){
          case railway_interfaces__msg__TrackObjectConfig__CONFIG_TYPE_LOCOMOTIVE:
            switch(track_object->protocol){
              case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1:
              case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2:
              case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC:
//                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MFX:
                p_iterator->address = track_object->address;
//                p_iterator->protocol = track_object->protocol;
                p_iterator++;
                break;
            }
            break;
        }
#endif
      }
#if 0

      p_turnouts = p_iterator;
      // Count the turnouts & locomotives
      for (size_t i = 0; i < track_config->track_objects.size; ++i) {
          const railway_interfaces__msg__TrackObjectConfig * track_object = 
        &track_config->track_objects.data[i];
        switch(track_object->config_type){
          case railway_interfaces__msg__TrackObjectConfig__CONFIG_TYPE_TURNOUT:
            switch(track_object->protocol){
              case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1:
              case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2:
              case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC:
                p_iterator->address = track_object->address;
//                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MFX:
                p_iterator->protocol = track_object->protocol;
                p_iterator++;
                break;
            }
            break;
        }
      }
      p_turnout_status = (bool *)p_iterator;
      if(number_of_turnouts)
        memset(p_turnout_status, 0, number_of_turnouts * sizeof(bool));
      *p_number_of_active_locomotives = number_of_locomotives;
      *p_number_of_active_turnouts = number_of_turnouts;
      *p_eeprom_programmed = EEPROM_PROGRAMMED_TAG;
      EEPROM.commit();
      track_config_enable_flag = false;
      dumpConfiguration();
#endif
#if 0
      DEBUG_PRINT("p_eeprom_programmed            = 0x%08x\n", p_eeprom_programmed);
      DEBUG_PRINT("p_number_of_active_locomotives = 0x%08x\n", p_number_of_active_locomotives);
      DEBUG_PRINT("p_number_of_active_turnouts    = 0x%08x\n", p_number_of_active_turnouts);
      DEBUG_PRINT("p_locomtives                   = 0x%08x\n", p_locomtives);
      DEBUG_PRINT("p_turnouts                     = 0x%08x\n", p_turnouts);
      DEBUG_PRINT("p_turnout_status               = 0x%08x\n", p_turnout_status);
      DEBUG_PRINT("number_of_locomotives %i\n", number_of_locomotives);
      DEBUG_PRINT("number_of_locomotives %i\n", number_of_turnouts);
#endif
    }
    tft_printf(ST77XX_MAGENTA, "Track\nConfiguration\nStored\n");
  }
  else
    dumpConfiguration();
}
