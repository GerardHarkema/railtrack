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

void track_config_callback(const void * msgin){
  
  const railway_interfaces__msg__TrackConfig * track_config = (const railway_interfaces__msg__TrackConfig *)msgin;

// Check if the message contains track objects
  if (!track_config->track_objects.data) {
      Serial.printf("No track objects in the received TrackConfig message\n");
      return;
  }

  // Iterate through the array of track objects
  for (size_t i = 0; i < track_config->track_objects.size; ++i) {
      const railway_interfaces__msg__TrackObjectConfig * track_object = 
          &track_config->track_objects.data[i];

      // Access fields of the TrackObjectConfig
      Serial.printf("Track Object %i\n", i);
  }

  tft_printf(ST77XX_MAGENTA, "Track config\received\n");

  Serial.printf("Configuration received\n");
  if(track_config_enable_flag){
    Serial.printf("Configuration accepted\n");
  }

}
