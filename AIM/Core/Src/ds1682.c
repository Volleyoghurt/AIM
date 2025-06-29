/*
 * ds1682.c
 *
 *  Created on: Jun 28, 2025
 *      Author: basge
 */

#include "ds1682.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;


/*----------------------------------------------------------------------------*/
/*— I2C DS1682 Runtime communicatie functies
/*----------------------------------------------------------------------------*/


/**
 * Lees één byte uit een DS1682-register via I²C
 * @param addr  7-bit I²C-adres van de DS1682 (linksshifted <<1 voor HAL)
 * @param reg   8-bit registerpointer binnen de DS1682
 * @return      Gelezen byte, of 0 bij communicatie­fout (foutmelding via UART)
 */
uint8_t DS1682_ReadRegister(uint8_t addr, uint8_t reg) {
    uint8_t val = 0;
    // Probeer 1 byte te lezen uit register 'reg'
    if (HAL_I2C_Mem_Read(&hi2c1,
                         addr,                  // slaveadres met R/W-bit
                         reg,                   // registerpointer
                         I2C_MEMADD_SIZE_8BIT,  // registeradresgrootte
                         &val,                  // ontvangbuffer
                         1,                     // aantal bytes
                         HAL_MAX_DELAY) != HAL_OK)
    {
        // Bij fout: log melding via UART
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "DS1682: Adres 0x%02X, geen I2C-communicatie\r\n",
                           (addr >> 1));  // shift terug naar 7-bit adres
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }
    return val;
}

/**
 * Schrijf één byte naar een DS1682-register via I²C
 * @param addr   7-bit I²C-adres van de DS1682 (linksshifted <<1 voor HAL)
 * @param reg    8-bit registerpointer binnen de DS1682
 * @param value  Byte-waarde om te schrijven
 * @return       HAL_OK bij succes, anders HAL-foutcode
 */
HAL_StatusTypeDef DS1682_WriteRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    // Schrijf precies 1 byte naar register 'reg'
    return HAL_I2C_Mem_Write(&hi2c1,
                             addr,                  // slaveadres
                             reg,                   // registerpointer
                             I2C_MEMADD_SIZE_8BIT,  // registeradresgrootte
                             &value,                // pointer naar data
                             1,                     // aantal bytes
                             HAL_MAX_DELAY);        // blocking timeout
}

/**
 * Lees de 32-bit elapsed-time teller van de DS1682
 * @param addr  7-bit I²C-adres van de DS1682 (linksshifted <<1 voor HAL)
 * @return      32-bit counter (LSB = laagste byte van reg LSB)
 */
uint32_t DS1682_ReadElapsedTime(uint8_t addr) {
    uint8_t data[4];
    // Lees 4 opeenvolgende bytes vanaf het LSB-register
    HAL_I2C_Mem_Read(&hi2c1,
                     addr,
                     DS1682_REG_ELAPSED_TIME_LSB,  // eerste register
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     sizeof(data),
                     HAL_MAX_DELAY);
    // Combineer tot 32-bit waarde (LSB eerst)
    return ((uint32_t)data[3] << 24) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[1] <<  8) |
            (uint32_t)data[0];
}

/**
 * Lees de 16-bit event-counter uit de DS1682
 * @param addr  7-bit I²C-adres van de DS1682 (linksshifted <<1 voor HAL)
 * @return      16-bit tellerwaarde (MSB<<8 | LSB)
 */
uint16_t DS1682_ReadEventCounter(uint8_t addr) {
    uint8_t data[2];
    // Lees 2 bytes vanaf het event-counter-register
    HAL_I2C_Mem_Read(&hi2c1,
                     addr,
                     DS1682_REG_EVENTCOUNTER_LSB,
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     sizeof(data),
                     HAL_MAX_DELAY);
    // Combineer tot 16-bit waarde
    return (uint16_t)((data[1] << 8) | data[0]);
}

/**
 * Converteer elapsed-seconds naar uren/minuten en toon via UART
 * @param addr  7-bit I²C-adres van de DS1682 (linksshifted <<1 voor HAL)
 *
 * Roept DS1682_ReadElapsedTime() aan, splitst in hrs/min,
 * en print "DS1682: Adres 0xXX: Tijd: HH uur : MM min".
 */


void DS1682_SecondsToHM_Display(uint8_t addr,uint8_t debug) {
    // Lees totaal aantal seconden
    uint32_t total_seconds = DS1682_ReadElapsedTime(addr);
    uint32_t hours   = total_seconds / 3600;        // volle uren
    uint32_t rem     = total_seconds % 3600;
    uint32_t minutes = rem / 60;                    // resterende minuten

    // Formatteer en verstuur bericht via UART
    if(debug == 1){
    	char buf[64];
    	int len = snprintf(buf, sizeof(buf),
                       "DS1682: Adres 0x%02X: Tijd: %02lu uur : %02lu min\r\n",
                       (addr >> 1), hours, minutes);
    	HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }
}

