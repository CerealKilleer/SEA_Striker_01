#include "wifi_lib.h"

esp_err_t dev_wifi_init(void)
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_err_t ret = esp_wifi_start();

    if (ret != ESP_OK)
    {
        ESP_LOGE("WIFI_LIB", "Failed to start Wi-Fi: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGI("WIFI_LIB", "Wi-Fi started successfully");
        esp_wifi_connect();
    }
    return ESP_OK;
}

void get_ip_address(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL)
    {
        ESP_LOGE("WIFI_LIB", "Failed to get netif handle");
        return;
    }

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(netif, &ip_info);
    ESP_LOGI("WIFI_LIB", "IP Address: " IPSTR, IP2STR(&ip_info.ip));
}