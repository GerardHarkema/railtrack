#include <Arduino.h>
#include "measurements.h"

#define CURRENT_MEASUREMENT_PIN         34
#define VOLTAGE_MEASUREMENT_PIN         39
#define TEMPERATURE_MEASUREMENT_PIN     35
#define CURRENT_MEASUREMENT_SHUNT_VALUE (double)10000.0 // Ohms

#define VOLTAGE_SCALING                 (11.0/1.0)/1000.0 //mV scale
#define IBT_2_CURRENT_SCALING           4.154

#define CURRENT_SCALING                 IBT_2_CURRENT_SCALING
#define TMP36_OFFSET_mV                 500.0
#define TMP36_SCALING                    0.1


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

    //DEBUG_PRINT("ADC Resolution = %i", analogReadResolution());

}




void Measurements::loop(){

    adc_current_last_value = analogReadMilliVolts(CURRENT_MEASUREMENT_PIN);
    adc_current_buffer[adc_current_index++] = adc_current_last_value;
    if(adc_current_index >= INTEGRATION_SIZE)adc_current_index = 0;

    adc_voltage_last_value = analogReadMilliVolts(VOLTAGE_MEASUREMENT_PIN);
    adc_voltage_buffer[adc_voltage_index++] = adc_voltage_last_value;
    if(adc_voltage_index >= INTEGRATION_SIZE)adc_voltage_index = 0;

    adc_temperature_last_value = analogReadMilliVolts(TEMPERATURE_MEASUREMENT_PIN);
    adc_temperature_buffer[adc_temperature_index++] = adc_temperature_last_value;
    if(adc_temperature_index >= INTEGRATION_SIZE)adc_temperature_index = 0;
    DEBUG_PRINT("Analog value %i mV\n", adc_current_current_value);
}

float Measurements::getCurrent(){
    int total_value = 0;
    for(int i = 0; i < INTEGRATION_SIZE; i++) total_value += adc_current_buffer[i];
    return (((float)total_value/INTEGRATION_SIZE)/CURRENT_MEASUREMENT_SHUNT_VALUE) * CURRENT_SCALING;// Amps
}

float Measurements::getLastCurrentMeasurement(){
    return (((float)adc_current_last_value/INTEGRATION_SIZE)/CURRENT_MEASUREMENT_SHUNT_VALUE) * CURRENT_SCALING;// Amps
}

float Measurements::getVoltage(){
    int total_value = 0;
    for(int i = 0; i < INTEGRATION_SIZE; i++) total_value += adc_voltage_buffer[i];
    return ((float)total_value/INTEGRATION_SIZE)*VOLTAGE_SCALING;// Volts
}

float Measurements::getLastVoltageMeasurement(){
    return ((float)adc_voltage_last_value/INTEGRATION_SIZE)*VOLTAGE_SCALING;// Volts
}


float Measurements::getTemperature(){
    int total_value = 0;
    for(int i = 0; i < INTEGRATION_SIZE; i++) total_value += adc_temperature_buffer[i];
    return (((float)total_value/INTEGRATION_SIZE)-TMP36_OFFSET_mV)*TMP36_SCALING; // Degrees Celsius
}

float Measurements::getLastTemperatureMeasurement(){
    return (((float)adc_temperature_last_value/INTEGRATION_SIZE)-TMP36_OFFSET_mV)*TMP36_SCALING; // Degrees Celsius
}