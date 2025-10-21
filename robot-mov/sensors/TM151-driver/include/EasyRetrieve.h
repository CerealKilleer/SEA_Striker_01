#ifndef _EASYRETRIEVE_H_
#define _EASYRETRIEVE_H_

static const char* TAG_TM151 = "TM151";     ///< Tag for TM151

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"

#include "EasyObjectDictionary.h"
#include "EasyProfile.h"

#include "platform_esp32s3.h" // Include the platform-specific header file for ESP32-S3
#include "EasyRetrieve.h"

/**
 * @brief Initialize the TM151 sensor
 * @param myUART: UART object
 * @param baudrate: Baudrate for UART communication
 * @param buffer_size: Buffer size for UART communication
 * @param tx_pin: GPIO pin for UART TX
 * @param rx_pin: GPIO pin for UART RX
 */
void tm151_init(uart_t *myUART, uint32_t baudrate, uint32_t buffer_size, uint8_t tx_pin, uint8_t rx_pin);

/**
 * @brief Called every time when new serial data is received
 */
void SerialPort_DataGet(uart_t* myUART);

/**
 * @brief Called every time when new serial data is received for raw acceleration data
 * @param myUART: UART object
 */
void SerialPort_DataGet_RawAcc(uart_t* myUART);

/**
 * @brief Called every time when new serial data is received
 * @param myUART: UART object
 * @param rawAcc: Pointer to store the raw acceleration data
 */
void SerialPort_DataReceived_RawAcc(uart_t* myUART, float* rawAcc);

/**
 * @brief Called every time when new serial data is received for raw yaw
 * @param myUART: UART object
 */
void SerialPort_DataGet_RawYaw(uart_t* myUART);

/**
 * @brief Called every time when new serial data is received for raw yaw
 * @param myUART: UART object
 * @param rawYaw: Pointer to store the raw yaw data
 */
void SerialPort_DataReceived_RawYaw(uart_t* myUART, float* rawYaw);

#endif