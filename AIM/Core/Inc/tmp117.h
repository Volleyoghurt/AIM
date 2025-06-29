#ifndef TMP117_H
#define TMP117_H

#include "stm32l4xx_hal.h"  // Pas aan voor jouw STM32-serie

// I2C-adres van de TMP117 (standaard 7-bit adres: 0x48)
#define TMP117_I2C_ADDR   (0x48 << 1)  // HAL expects 8-bit address

// Registeradressen volgens datasheet
#define TMP117_REG_TEMP_RESULT   0x00
#define TMP117_REG_CONFIG        0x01
#define TMP117_REG_T_HIGH_LIMIT  0x02
#define TMP117_REG_T_LOW_LIMIT   0x03
#define TMP117_REG_EEPROM_UL     0x04
#define TMP117_REG_EEPROM1       0x05
#define TMP117_REG_EEPROM2       0x06
#define TMP117_REG_TEMP_OFFSET   0x07
#define TMP117_REG_DEVICE_ID     0x0F

// Functieprototypes
uint16_t TMP117_ReadRegister(uint8_t addr, uint8_t reg);
HAL_StatusTypeDef TMP117_WriteRegister(uint8_t addr, uint8_t reg, uint16_t value);
uint16_t TMP117_ReadTemperatureC(uint8_t addr, uint8_t reg,uint8_t debug);
void TMP_SetAlarmTemp(uint8_t addr, uint8_t reg, float temp);
void TMP117_Display_Register(uint8_t addr, uint16_t Register);

#endif // TMP117_H
