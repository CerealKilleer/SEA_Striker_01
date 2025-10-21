
#include <errno.h>
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_err.h"

#ifndef _WIFI_LIB_H_
#define _WIFI_LIB_H_

#define WIFI_SSID "Nelson's S24"//"Howlers - UdeA"
#define WIFI_PASS "1234567890" //"9876543210"
#define WIFI_MAXIMUM_RETRY 5
#define PORT 3333


/**
 * @brief Initialize WiFi in soft Access point mode
 * 
 * This function initializes the WiFi in soft access point mode, sets the SSID and password,
 * and starts the WiFi connection process.
 * 
 * It uses the ESP-IDF WiFi API to configure the WiFi settings and connect to the specified network. 
 * It also handles the initialization of the NVS flash storage and the event loop for WiFi events.
 * 
 * @note This function should be called in the `app_main` function to start the WiFi connection process.
 * @note Make sure to replace `WIFI_SSID` and `WIFI_PASS` with your actual WiFi credentials.
 * 
 * 
 * @return esp_err_t Returns `ESP_OK` on success, or an error code on failure.
 */

esp_err_t dev_wifi_init(void);

/**
 * @brief Get the IP address of the WiFi interface
 * 
 * This function retrieves the IP address assigned to the WiFi interface and logs it.
 * 
 * It uses the ESP-IDF API to get the IP information from the network interface and logs it using ESP_LOGI.
 */
void get_ip_address(void);

#endif // _WIFI_LIB_H_
