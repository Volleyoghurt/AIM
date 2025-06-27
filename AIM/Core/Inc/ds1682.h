/*
 * ds1682.h
 *
 *  Created on: Jun 10, 2025
 *      Author: basge
 */

#ifndef INC_DS1682_H_
#define INC_DS1682_H_

#ifndef DS1682_H
#define DS1682_H

// DS1682 7-bit I2C-adres (standaard 0x6B)
#define DS1682_I2C_ADDR   (0x6B << 1)  // STM32 HAL expects 8-bit address

// Registeradressen (volgens datasheet)
#define DS1682_REG_CONFIG             0x00
#define DS1682_REG_ALARM_LSB	      0x01
#define DS1682_REG_ALARM_LMB		  0x02
#define DS1682_REG_ALARM_HMB		  0x03
#define DS1682_REG_ALARM_MSB		  0x04
#define DS1682_REG_ELAPSED_TIME_LSB   0x05
#define DS1682_REG_ELAPSED_TIME_LMB   0x06
#define DS1682_REG_ELAPSED_TIME_HMB   0x07
#define DS1682_REG_ELAPSED_TIME_MSB   0x08
#define DS1682_REG_EVENTCOUNTER_LSB	  0x09
#define DS1682_REG_EVENTCOUNTER_MSB	  0x0A
#define DS1682_REG_RESET_ENABLE       0x1D

// Functieprototypes

void DS1682_Init(uint8_t addr,uint8_t config);
HAL_StatusTypeDef DS1682_WriteRegister(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t DS1682_ReadRegister(uint8_t addr, uint8_t reg);
uint32_t DS1682_ReadElapsedTime(uint8_t addr);
uint32_t DS1682_ReadPowerOnTime(uint8_t addr);
uint16_t DS1682_ReadEventCounter(uint8_t addr);
void DS1682_SecondsToHM_Display(uint8_t addr,uint8_t debug);


#endif // DS1682_H


#endif /* INC_DS1682_H_ */
