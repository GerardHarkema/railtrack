#ifndef MEASUREMENTS
#define MEASUREMENTS

#define INTEGRATION_SIZE    10

class Measurements{
public:
    void begin();
    void loop();
    float getCurrent();
    float getLastCurrentMeasurement();
    float getVoltage();
    float getLastVoltageMeasurement();
    float getTemperature();
    float getLastTemperatureMeasurement();
    TaskHandle_t scheduler_task_h = NULL;
private:
    static void scheduler_task(void *pvParameters);
    int adc_current_buffer[INTEGRATION_SIZE];
    int adc_current_last_value;
    int adc_current_index = 0;

    int adc_voltage_buffer[INTEGRATION_SIZE];
    int adc_voltage_last_value;
    int adc_voltage_index = 0;

    int adc_temperature_buffer[INTEGRATION_SIZE];
    int adc_temperature_last_value;
    int adc_temperature_index = 0;
};

#endif //MEASUREMENTS