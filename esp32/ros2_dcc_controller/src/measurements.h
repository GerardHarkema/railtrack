#ifndef MEASUREMENTS
#define MEASUREMENTS

#define INTEGRATION_SIZE    10

class Measurements{
public:
    void begin();
    void loop();
    float getCurrent();
    float getLastCurrentValue();
    float getVoltage();
    float getTemperature();
    TaskHandle_t scheduler_task_h = NULL;
private:
    static void scheduler_task(void *pvParameters);
    int adc_buffer[INTEGRATION_SIZE];
    int adc_last_value;
    int adc_index = 0;

};

#endif //MEASUREMENTS