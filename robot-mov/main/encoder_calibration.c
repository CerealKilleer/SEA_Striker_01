/**
 * @file main.c
 * @author MaverickST
 * @brief Main file for the AS5600 library
 * @details This file contains the main function and the calibration process for the AS5600 sensor.
 * @version 0.1
 * @date 2025-04-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <stdio.h>

#include "as5600_lib.h"

#define I2C_MASTER_SCL_GPIO 4       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 5       /*!< gpio number for I2C master data  */
#define AS5600_OUT_GPIO 6           /*!< gpio number for OUT signal */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */

AS5600_t gAs5600;
/**
 * @brief This task configures one AS5600 encoder, feel free to change the configuration options if you need it.
 * You must know that this task configures just one encoder, you need to desconnect and connect each encoder that you
 * need to configure. 
 */
void app_main(void)
{

    vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for the system to stabilize

    ///< ---------------------- AS5600 -------------------
    AS5600_Init(&gAs5600, I2C_MASTER_NUM, I2C_MASTER_SCL_GPIO, I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO);

    // Set some configurations to the AS5600
    AS5600_config_t conf = {
        .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
        .HYST = AS5600_HYSTERESIS_2LSB, ///< Hysteresis 2LSB
        .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
        .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
        .SF = AS5600_SLOW_FILTER_8X, ///< Slow filter 8x
        .FTH = AS5600_FF_THRESHOLD_6LSB, ///< Fast filter threshold 6LSB
        .WD = AS5600_WATCHDOG_OFF, ///< Watchdog off
    };
    AS5600_config_t confAux;
    uint16_t start_position, stop_position;
    //Delete the below commentary if you want to configure a new encoder
    /*
    
    AS5600_SetConf(&gAs5600, conf); ///< Write new conf in CONF register (0x07)

    
    AS5600_GetConf(&gAs5600, &confAux); ///< Read the writead conf to know if was writed correctly
    assert(confAux.WORD == conf.WORD); ///< Assert that the configuration is set correctly
    
    ///< Read the configuration again y shows results. Must be the same
    uint16_t conf_reg;
    AS5600_ReadReg(&gAs5600, AS5600_REG_CONF_H, &conf_reg);
    printf("Configuration register readed: 0x%04X\n", conf_reg);
    printf("Configuration register written: 0x%04X\n", conf.WORD);

    ///< ------------- For calibration process. -------------
    AS5600_SetStartPosition(&gAs5600, 0x0000); ///< Set start position to 0 degrees
    AS5600_SetStopPosition(&gAs5600, 0x0FFF); ///< Set stop position to 360 degrees 12 bits
    
    
    AS5600_GetStopPosition(&gAs5600, &stop_position);   ///< Get stop position
    assert(stop_position != 0x0000); ///< Assert that the stop position is set correctly
    ///< -------------

    ///< Burn commands
    AS5600_BurnSettingCommand(&gAs5600); ///< TAKE CARE! This command just works once. It will burn the configuration to the EEPROM of the AS5600 sensor.
    AS5600_BurnAngleCommand(&gAs5600);   ///< TAKE CARE! This command just works three times. It will burn the start and end angles to the EEPROM of the AS5600 sensor.
    */
    while(1){
        ///< ------------- For calibration process. -------------
        
        AS5600_GetConf(&gAs5600, &confAux); ///< Get the configuration
        printf("Configuration: 0x%04X\tConfig. Read: 0x%04X\n", conf.WORD, confAux.WORD);

        AS5600_GetStartPosition(&gAs5600, &start_position); ///< Get start position
        AS5600_GetStopPosition(&gAs5600, &stop_position);   ///< Get stop position
        printf("Start position: 0x%04X\tStop position: 0x%04X\n", start_position, stop_position);
        ///< -------------
        vTaskDelay(1000 / portTICK_PERIOD_MS); ///< Delay for 1 second
    }
}