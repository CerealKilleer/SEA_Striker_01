#include "wifi_lib.h"

from_wifi_t from_wifi = {
    .type = LINE_MOVEMENT,
    .recived = false, // Inicializar como no recibido
    .direction = 0.0,
    .degrees = 0.0,
    .velocity = 0.0,
    .distance = 0.0,
    .radius = 0.0
};

esp_err_t dev_wifi_init(void) {
    ESP_LOGI("wifi lib", "Inicializando WiFi en modo Access Point...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }


    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = STA_SSID,
            .password = STA_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI("WIFI STATION", "Conecting to %s...", STA_SSID);
    esp_wifi_connect();
    
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

            // Convertir los parámetros a float y asignarlos a from_wifi
            from_wifi.type = LINE_MOVEMENT;
            from_wifi.recived = true; // Indicar que se ha recibido un comando
            from_wifi.direction = (strcmp(direction, "Backward") == 0) ? 0 : 1; // 0: backward, 1: forward
            from_wifi.degrees = atof(degrees);
            from_wifi.velocity = atof(velocity);
            from_wifi.distance = atof(distance);

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

            // Convertir los parámetros a float y asignarlos a from_wifi
            from_wifi.type = CIRCULAR_MOVEMENT;
            from_wifi.recived = true; // Indicar que se ha recibido un comando
            from_wifi.direction = (strcmp(direction, "ccw") == 0) ? 0 : 1; // 0: backward, 1: forward
            from_wifi.degrees = atof(degrees);
            from_wifi.velocity = atof(velocity);
            from_wifi.radius = atof(radius);
            from_wifi.distance = 0; // No se usa en circular movement

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

            // Convertir los parámetros a float y asignarlos a from_wifi
            from_wifi.type = SELF_ROTATION;
            from_wifi.recived = true; // Indicar que se ha recibido un comando
            from_wifi.direction = (strcmp(direction, "ccw") == 0) ? 0 : 1; // 0: ccw, 1: cw
            from_wifi.degrees = atof(degrees);
            from_wifi.velocity = atof(velocity);
            from_wifi.distance = 0; // No distance for self-rotation
            from_wifi.radius = 0; // No radius for self-rotation

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


