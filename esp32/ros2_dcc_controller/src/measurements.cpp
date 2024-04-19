#include <Arduino.h>
#include "measurements.h"


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

void Measurements::scheduler_task(void *pvParameters)
{
  Measurements *instance = static_cast<Measurements*>(pvParameters);
	while(1)
	{
        vTaskDelay(100);
        instance->loop();
	}
}

void Measurements::begin(){
    analogSetClockDiv(255);

    int app_cpu = xPortGetCoreID();
    xTaskCreatePinnedToCore(scheduler_task,
                        "measurement_task", 
                        4096,
                        this,
                        1,
                        &scheduler_task_h,
                        app_cpu);
  if(!scheduler_task_h){
    DEBUG_PRINT("Unable to start task\n");
  }

    //Serial.printf("ADC Resolution = %i", analogReadResolution());

}

int adc_current_value = 0;
void Measurements::loop(){

    adc_last_value = analogReadMilliVolts(34);
    adc_buffer[adc_index++] = adc_last_value;
    if(adc_index >= INTEGRATION_SIZE)adc_index = 0;

    //Serial.printf("Analog value %i mV\n", adc_current_value);
}

float Measurements::getCurrent(){
    int total_value = 0;
    for(int i = 0; i < INTEGRATION_SIZE; i++) total_value += adc_buffer[i];
    return ((float)total_value/INTEGRATION_SIZE)/1000.0;// Amps
}

float Measurements::getLastCurrentValue(){
    return ((float)adc_last_value/INTEGRATION_SIZE)/1000.0;
}

float Measurements::getVoltage(){

}
float Measurements::getTemperature(){

}