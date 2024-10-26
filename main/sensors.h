#define I2C_PORT I2C_NUM_0
#define PIN_SDA GPIO_NUM_9
#define PIN_SCL GPIO_NUM_8
#define I2C_FREQ_HZ 40000
#define SENSORS_TIMING_BUDGET_MS 33
#define SENSORS_COUNT 3
#define SENSORS_XSHUTS GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7

#include <stdbool.h>
#include <stdint.h>

void sensors_init();
void sensors_start_measuring();
void sensors_stop_measuring();
void sensors_get_values(uint16_t *values);
bool sensors_new_values_available();