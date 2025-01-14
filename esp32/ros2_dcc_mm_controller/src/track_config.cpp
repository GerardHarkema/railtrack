#include <Arduino.h>
#include "support.h"
#include "track_config.h"

//#define DEBUG
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


bool enoughNeededEeprom(int number_of_locomotives, int number_of_turnouts){
  int needed_eeprom_size;

  needed_eeprom_size += sizeof(uint16_t); // eeprom programmed
  needed_eeprom_size += sizeof(uint16_t); // number_of_turnouts
  needed_eeprom_size += sizeof(uint16_t); // number_of_locomotive
  needed_eeprom_size += sizeof(TRACK_OBJECT) * number_of_locomotives; // needed_eeprom_size of locomotive objects
  needed_eeprom_size += sizeof(TRACK_OBJECT) * number_of_turnouts; // needed_eeprom_size of turnout objects
  needed_eeprom_size += sizeof(bool) * number_of_turnouts; // needed_eeprom_size of turnout_status
  Serial.printf("needed_eeprom_size = %i\n", needed_eeprom_size);
  if(needed_eeprom_size > EEPROM_SIZE){
    Serial.printf("needed_eeprom_size to large, size = %i, max = %i\n", needed_eeprom_size, EEPROM_SIZE);
  }
  return (needed_eeprom_size > EEPROM_SIZE) ? false: true;
}

void track_config_callback(const void * msgin){

  const railway_interfaces__msg__TrackConfig * track_config = (const railway_interfaces__msg__TrackConfig *)msgin;
  int number_of_turnouts = 0;
  int number_of_locomotives = 0;

// Check if the message contains track objects
  if (!track_config->track_objects.data) {
      Serial.printf("No track objects in the received TrackConfig message\n");
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
              number_of_turnouts++;
              break;
          }
  }

  TRACK_OBJECT *p_iterator = p_locomtives;

  // Count the turnouts & locomotives
  for (size_t i = 0; i < track_config->track_objects.size; ++i) {
      const railway_interfaces__msg__TrackObjectConfig * track_object = 
          &track_config->track_objects.data[i];
          switch(track_object->config_type){
            case railway_interfaces__msg__TrackObjectConfig__CONFIG_TYPE_LOCOMOTIVE:
              p_iterator->address = track_object->address;
              switch(track_object->protocol){
                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_ROS:
                  break;
                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1:
                  p_iterator->protocol = MM1;
                  break;
                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2:
                  p_iterator->protocol = MM2;
                  break;
                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC:
                  p_iterator->protocol = DCC;
                  break;
                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MFX:
                  break;
              }
              p_iterator++;
              break;
          }
  }

  p_turnouts = p_iterator;
  // Count the turnouts & locomotives
  for (size_t i = 0; i < track_config->track_objects.size; ++i) {
      const railway_interfaces__msg__TrackObjectConfig * track_object = 
          &track_config->track_objects.data[i];
          switch(track_object->config_type){
            case railway_interfaces__msg__TrackObjectConfig__CONFIG_TYPE_TURNOUT:
              p_iterator->address = track_object->address;
              switch(track_object->protocol){
                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_ROS:
                  break;
                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM1:
                  p_iterator->protocol = MM1;
                  break;
                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MM2:
                  p_iterator->protocol = MM2;
                  break;
                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_DCC:
                  p_iterator->protocol = DCC;
                  break;
                case railway_interfaces__msg__TrackProtocolDefines__PROTOCOL_MFX:
                  break;
              }
              p_iterator++;
              break;
          }
  }
  p_turnout_status = (bool *)p_iterator;
  memset(p_turnout_status, 0, number_of_turnouts * sizeof(bool));



  Serial.printf("p_eeprom_programmed            = 0x%08x\n", p_eeprom_programmed);
  Serial.printf("p_number_of_active_locomotives = 0x%08x\n", p_number_of_active_locomotives);
  Serial.printf("p_number_of_active_turnouts    = 0x%08x\n", p_number_of_active_turnouts);
  Serial.printf("p_locomtives                   = 0x%08x\n", p_locomtives);
  Serial.printf("p_turnouts                     = 0x%08x\n", p_turnouts);
  Serial.printf("p_turnout_status               = 0x%08x\n", p_turnout_status);


  Serial.printf("number_of_locomotives %i\n", number_of_locomotives);
  Serial.printf("number_of_locomotives %i\n", number_of_turnouts);
  enoughNeededEeprom(number_of_locomotives, number_of_locomotives);

  tft_printf(ST77XX_MAGENTA, "Track configuration\nreceived\n");

  Serial.printf("Configuration\n received\n");
  if(track_config_enable_flag){
    Serial.printf("Configuration\n accepted\n");
  }
}
