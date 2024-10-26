#include "VL53L0X.h"
extern "C" {
#include "sensors.h"
}
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

gpio_num_t xshuts[] = {SENSORS_XSHUTS};
VL53L0X sensors[SENSORS_COUNT];
uint16_t sensor_values[SENSORS_COUNT];
bool new_values[SENSORS_COUNT];
bool sensors_enabled = false;

extern "C" void sensors_init(){
    for (int i = 0; i<sizeof(xshuts)/sizeof(gpio_num_t);i++){
        gpio_reset_pin(xshuts[i]);
        gpio_set_level(xshuts[i], 0);
        gpio_set_direction(xshuts[i], GPIO_MODE_OUTPUT);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    
    sensors[0].i2cMasterInit(PIN_SDA, PIN_SCL, 30000);

    for (int i = 0; i<SENSORS_COUNT;i++){
        gpio_set_level(xshuts[i], 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        sensors[i] = VL53L0X(I2C_PORT);
        sensors[i].init();
        VL53L0X_SetDeviceAddress(&sensors[i].vl53l0x_dev, i*2);
        sensors[i].vl53l0x_dev.i2c_address = i;
        VL53L0X_SetDeviceMode(&sensors[i].vl53l0x_dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        VL53L0X_StartMeasurement(&sensors[i].vl53l0x_dev);
    }
}

extern "C" void sensors_loop(){
    while (sensors_enabled) {
        for (int i = 0;i<SENSORS_COUNT;i++){
            uint8_t data_ready = false;
            VL53L0X_GetMeasurementDataReady(&sensors[i].vl53l0x_dev, &data_ready);
            if (data_ready){
                VL53L0X_RangingMeasurementData_t range_data;
                VL53L0X_GetRangingMeasurementData(&sensors[i].vl53l0x_dev, &range_data);
                VL53L0X_ClearInterruptMask(&sensors[i].vl53l0x_dev, 0);
                sensor_values[i] = range_data.RangeMilliMeter;
                new_values[i] = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

extern "C" void sensors_start_measuring(){
    sensors_enabled = true;
    xTaskCreate((TaskFunction_t)&sensors_loop, "sensors_task", 2048, NULL, 5, NULL);
}

extern "C" void sensors_stop_measuring(){
    sensors_enabled = false;
}

extern "C" void sensors_get_values(uint16_t *values){
   memcpy(values,sensor_values,SENSORS_COUNT*2);
}

extern "C" bool sensors_new_values_available(){
    for (int i = 0; i<SENSORS_COUNT;i++){
        if (new_values[i] == false){
            return false;
        }
    }
    memset(new_values,0,SENSORS_COUNT);
    return true;
}