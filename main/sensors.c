#include "sensors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

gpio_num_t xshuts[] = {SENSORS_XSHUTS};
VL53L0X_Dev_t sensors[SENSORS_COUNT] = {0};
uint16_t sensor_values[SENSORS_COUNT] = {0};
bool new_values[SENSORS_COUNT] = {0};
bool sensors_enabled = false;


void _vl53_error_check_failed(VL53L0X_Error rc, const char *file, int line, const char *function, const char *expression)
{
    printf("VL53_ERROR_CHECK failed: VL53L0X_Error %i", rc);
#ifdef CONFIG_ESP_ERR_TO_NAME_LOOKUP
    char buf[32];
    VL53L0X_GetPalErrorString(rc, buf);
    printf(" (%s)", buf);
#endif //CONFIG_ESP_ERR_TO_NAME_LOOKUP
    printf(" at %p\n", __builtin_return_address(0));
    printf("file: \"%s\" line %d\nfunc: %s\nexpression: %s\n", file, line, function, expression);
    abort();
}

#define VL53_ERROR_CHECK(x) do {                                    \
    VL53L0X_Error err_rc_ = (x);                                    \
    if (unlikely(err_rc_ != VL53L0X_ERROR_NONE)) {                  \
        _vl53_error_check_failed(err_rc_, __FILE__, __LINE__,       \
                                __ASSERT_FUNC, #x);                 \
    }                                                               \
} while(0)


void setup_i2c(){
    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PIN_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = PIN_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ;
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

void init_sensor(VL53L0X_Dev_t* sensor){
    uint8_t isApertureSpads;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t VhvSettings;

    sensor->i2c_port_num = I2C_PORT;
    sensor->i2c_address = 0x29;
    VL53_ERROR_CHECK(VL53L0X_DataInit(sensor));
    VL53_ERROR_CHECK(VL53L0X_StaticInit(sensor));
    VL53_ERROR_CHECK(VL53L0X_PerformRefSpadManagement(sensor, &refSpadCount, &isApertureSpads));
    VL53_ERROR_CHECK(VL53L0X_PerformRefCalibration(sensor, &VhvSettings, &PhaseCal));
    VL53_ERROR_CHECK(VL53L0X_SetDeviceMode(sensor, VL53L0X_DEVICEMODE_SINGLE_RANGING));
    VL53_ERROR_CHECK(VL53L0X_SetGpioConfig(sensor, 0,
                              VL53L0X_DEVICEMODE_SINGLE_RANGING,
                              VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
                              VL53L0X_INTERRUPTPOLARITY_LOW));
    VL53_ERROR_CHECK(VL53L0X_SetMeasurementTimingBudgetMicroSeconds(sensor, 33000));
}

void sensors_init(){
    for (int i = 0; i<sizeof(xshuts)/sizeof(gpio_num_t);i++){
        ESP_ERROR_CHECK(gpio_reset_pin(xshuts[i]));
        ESP_ERROR_CHECK(gpio_set_level(xshuts[i], 0));
        ESP_ERROR_CHECK(gpio_set_direction(xshuts[i], GPIO_MODE_OUTPUT));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    setup_i2c();

    for (int i = 0; i<SENSORS_COUNT;i++){
        ESP_ERROR_CHECK(gpio_set_level(xshuts[i], 1));
        vTaskDelay(pdMS_TO_TICKS(10));
        init_sensor(&sensors[i]);
        VL53_ERROR_CHECK(VL53L0X_SetDeviceAddress(&sensors[i], i*2));
        sensors[i].i2c_address = i;
        VL53_ERROR_CHECK(VL53L0X_SetDeviceMode(&sensors[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING));
        VL53_ERROR_CHECK(VL53L0X_StartMeasurement(&sensors[i]));
    }
}

void sensors_loop(){
    while (sensors_enabled) {
        for (int i = 0;i<SENSORS_COUNT;i++){
            uint8_t data_ready = false;
            VL53_ERROR_CHECK(VL53L0X_GetMeasurementDataReady(&sensors[i], &data_ready));
            if (data_ready){
                VL53L0X_RangingMeasurementData_t range_data;
                VL53_ERROR_CHECK(VL53L0X_GetRangingMeasurementData(&sensors[i], &range_data));
                VL53_ERROR_CHECK(VL53L0X_ClearInterruptMask(&sensors[i], 0));
                sensor_values[i] = range_data.RangeMilliMeter;
                new_values[i] = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void sensors_start_measuring(){
    sensors_enabled = true;
    xTaskCreate((TaskFunction_t)&sensors_loop, "sensors_task", 2048, NULL, 5, NULL);
}

void sensors_stop_measuring(){
    sensors_enabled = false;
}

void sensors_get_values(uint16_t *values){
   memcpy(values,sensor_values,SENSORS_COUNT*2);
}

bool sensors_new_values_available(){
    for (int i = 0; i<SENSORS_COUNT;i++){
        if (new_values[i] == false){
            return false;
        }
    }
    memset(new_values,0,SENSORS_COUNT);
    return true;
}