#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#include "EasyRetrieve.h"

///<------------- TM151 configuration ----------------
#define TM151_UART_TX 17                          ///< Gpio pin for UART TX
#define TM151_UART_RX 18                          ///< GPIO pin for UART RX
#define TM151_UART_BAUDRATE 115600                ///< Baudrate for UART communication
#define TM151_BUFFER_SIZE 1024                       ///< Buffer size for UART communication

typedef struct {
    float ax_b;     ///< Body-frame X acceleration [m/s^2]
    float ay_b;     ///< Body-frame Y acceleration [m/s^2]
    float z_omega;  ///< Gyroscope measurement [rad/s]
} imu_input_t;

typedef struct {
    uart_t * myUART;              ///< UART object for TM151 IMU
    imu_input_t * imu_data;       ///< IMU data 
} imu_params_t;



TaskHandle_t xIMUTaskHandle;


void timer_isr(void *args) {
    xTaskNotifyFromISR(*((TaskHandle_t *)args), 0x01, eSetBits, 0);
}

void vTaskIMU(void * pvParameters);

void app_main(void){

    static uart_t myUART;
    static imu_input_t imu_data;
    imu_params_t imu_params = {
        .myUART = &myUART,
        .imu_data = &imu_data};
    tm151_init(&myUART, TM151_UART_BAUDRATE, TM151_BUFFER_SIZE, TM151_UART_TX, TM151_UART_RX); ///< Initialize the TM151 sensor


    if (xTaskCreate(vTaskIMU, "imu_task", 4096, 
                (void *)&imu_params, 21, &xIMUTaskHandle) != pdPASS) {

                ESP_LOGE("IMU task create", "Failed to create");
                return; 
    }
    esp_timer_handle_t timer_handle;

    const esp_timer_create_args_t timer_args = {
        .callback = timer_isr,
        .arg = (void *)&xIMUTaskHandle,
        .dispatch_method = ESP_TIMER_ISR,
        .name = "timer isr",
        .skip_unhandled_events = false
    };

    esp_timer_create(&timer_args, &timer_handle);
    esp_timer_start_periodic(timer_handle, 20000);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void vTaskIMU(void * pvParameters) {

    imu_params_t *params = (imu_params_t *)pvParameters; ///< IMU parameters structure
    imu_input_t *imu_data = params->imu_data; ///< IMU data structure
    uart_t *myUART = params->myUART; ///< UART object for TM151 IMU

    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();
    // Get task name
    const char *task_name = pcTaskGetName(xTask);

    float acceleration[3], gyro_z, yaw;
    int cont = 0;
    
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Read acceleration data from TM151 IMU
        SerialPort_DataReceived_RawAcc(myUART, acceleration, &gyro_z); ///< Read acceleration and gyroscope data from TM151 IMU

        // Read yaw data from TM151 IMU
        SerialPort_DataReceived_RawYaw(myUART, &yaw); ///< Read yaw data from
        
        // Estimate the velocity using IMU data
        // estimate_velocity_imu(imu_data, acceleration[0], SAMPLE_TIME / 1000.0f); ///< Estimate the velocity using IMU data
        cont++;
        //Acceleration: [\t%.2f,\t%.2f,\t%.2f]\t Gyro: [\t%.2f,\t%.2f,\t%.2f]
        if(cont >= 1){
            cont = 0;
            ESP_LOGI(task_name, "IMU: %.2f, %.2f, %.2f, %.2f, %.2f", 
                     acceleration[0], acceleration[1], acceleration[2],
                     gyro_z, yaw);
        }
        // Log every 100ms because of the ESP_LOGI overhead
        /*ESP_LOGI(task_name, "Acceleration: [\t%.2f,\t%.2f,\t%.2f]\t Yaw: %.2f", acceleration[0], acceleration[1], acceleration[2], yaw);*/
    }
}

void estimate_velocity_imu(imu_input_t *imu_data, float time_interval){
    // Convertir g a m/s^2
    float acc_x_ms2 = imu_data->ax_b * 9.794f;
    float acc_y_ms2 = imu_data->ay_b * 9.794f;


}