#include "wifi_lib.h"

esp_err_t dev_wifi_init(void) {
    ESP_LOGI("wifi lib", "Inicializando WiFi en modo Access Point...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .password = AP_PASSWORD,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen(AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI("Wifi lib", "AP iniciado. SSID: %s | Password: %s", AP_SSID, AP_PASSWORD);
    
    return ESP_OK;
}


httpd_handle_t start_server (void) {

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_line_movement = {
            .uri = "/lineMovement",
            .method = HTTP_GET,
            .handler = line_movement_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_line_movement);

        httpd_uri_t uri_circular_movement = {
            .uri = "/circularMovement",
            .method = HTTP_GET,
            .handler = circular_movement_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_circular_movement);

        httpd_uri_t uri_self_rotation = {
            .uri = "/selfRotation",
            .method = HTTP_GET,
            .handler = selfRotation_movement_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_self_rotation);        

        ESP_LOGI("Wifi lib", "Servidor web iniciado en modo AP");
    }

    return server;

}

esp_err_t line_movement_handler(httpd_req_t *req) {
    char param[128];
    ESP_LOGI("Wifi lib", ">> Entró al handler /lineMovement");

    // Configurar encabezados CORS
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (httpd_req_get_url_query_str(req, param, sizeof(param)) == ESP_OK) {
        ESP_LOGI("Wifi lib", "Query string: %s", param);

        char direction[16], degrees[8], velocity[8], distance[8];

        if (httpd_query_key_value(param, "direction", direction, sizeof(direction)) == ESP_OK &&
            httpd_query_key_value(param, "degrees", degrees, sizeof(degrees)) == ESP_OK &&
            httpd_query_key_value(param, "velocity", velocity, sizeof(velocity)) == ESP_OK &&
            httpd_query_key_value(param, "distance", distance, sizeof(distance)) == ESP_OK) {

            ESP_LOGI("Wifi lib", ">> Params OK");
            ESP_LOGI("Wifi lib", "Direction: %s", direction);
            ESP_LOGI("Wifi lib", "Degrees: %s", degrees);
            ESP_LOGI("Wifi lib", "Velocity: %s", velocity);
            ESP_LOGI("Wifi lib", "Distance: %s", distance);

            httpd_resp_sendstr(req, "Command received");
            return ESP_OK;
        } else {
            ESP_LOGE("Wifi lib", "Error al obtener parámetros");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing params");
            return ESP_FAIL;
        }

    } else {
        ESP_LOGE("Wifi lib", "No se pudo obtener la query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No query string");
        return ESP_FAIL;
    }
}

esp_err_t circular_movement_handler(httpd_req_t *req) {
    char param[128];
    ESP_LOGI("Wifi lib", ">> Entró al handler /circularMovement");

    // Configurar encabezados CORS
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (httpd_req_get_url_query_str(req, param, sizeof(param)) == ESP_OK) {
        ESP_LOGI("Wifi lib", "Query string: %s", param);

        char direction[16], degrees[8], velocity[8], radius[8];

        if (httpd_query_key_value(param, "direction", direction, sizeof(direction)) == ESP_OK &&
            httpd_query_key_value(param, "degrees", degrees, sizeof(degrees)) == ESP_OK &&
            httpd_query_key_value(param, "velocity", velocity, sizeof(velocity)) == ESP_OK &&
            httpd_query_key_value(param, "distance", radius, sizeof(radius)) == ESP_OK) {

            ESP_LOGI("Wifi lib", ">> Params OK");
            ESP_LOGI("Wifi lib", "Direction: %s", direction);
            ESP_LOGI("Wifi lib", "Degrees: %s", degrees);
            ESP_LOGI("Wifi lib", "Velocity: %s", velocity);
            ESP_LOGI("Wifi lib", "Radius: %s", radius);

            httpd_resp_sendstr(req, "Command received");
            return ESP_OK;
        } else {
            ESP_LOGE("Wifi lib", "Error obtaining parameters");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing params");
            return ESP_FAIL;
        }

    } else {
        ESP_LOGE("Wifi lib", "Couldn't get the query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No query string");
        return ESP_FAIL;
    }
}

esp_err_t selfRotation_movement_handler(httpd_req_t *req) {
    char param[128];
    ESP_LOGI("Wifi lib", ">> Entró al handler /selfRotation");

    // Configurar encabezados CORS
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (httpd_req_get_url_query_str(req, param, sizeof(param)) == ESP_OK) {
        ESP_LOGI("Wifi lib", "Query string: %s", param);

        char direction[16], degrees[8], velocity[8];

        if (httpd_query_key_value(param, "direction", direction, sizeof(direction)) == ESP_OK &&
            httpd_query_key_value(param, "degrees", degrees, sizeof(degrees)) == ESP_OK &&
            httpd_query_key_value(param, "velocity", velocity, sizeof(velocity)) == ESP_OK ) {

            ESP_LOGI("Wifi lib", ">> Params OK");
            ESP_LOGI("Wifi lib", "Direction: %s", direction);
            ESP_LOGI("Wifi lib", "Degrees: %s", degrees);
            ESP_LOGI("Wifi lib", "Velocity: %s", velocity);

            httpd_resp_sendstr(req, "Command received");
            return ESP_OK;
        } else {
            ESP_LOGE("Wifi lib", "Error obtaining parameters");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing params");
            return ESP_FAIL;
        }

    } else {
        ESP_LOGE("Wifi lib", "Couldn't get the query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No query string");
        return ESP_FAIL;
    }
}


