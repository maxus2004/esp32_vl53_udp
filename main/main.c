#include "sensors.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "wifi.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

bool send_only_new_measurements = true;

void app_main() {
    nvs_flash_init();
    wifi_init();

    sensors_init();
    sensors_start_measuring();

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr("192.168.1.8");
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(5555);

    while(true){
        if(send_only_new_measurements){
            while(!sensors_new_values_available()){
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        uint16_t values[SENSORS_COUNT];
        sensors_get_values(values);
        sendto(sock, values, SENSORS_COUNT*2, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    }
}
