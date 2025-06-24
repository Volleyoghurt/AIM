/*
 * INA238.h
 *
 *  Created on: Jun 13, 2025
 *      Author: basge
 */

#ifndef INC_INA238_H_
#define INC_INA238_H_

// I2C-adres van de INA238 (7-bit adres links verschoven voor HAL)
#define INA238_I2C_ADDR   (0x40 << 1)

// Registeradressen
#define INA238_REG_CONFIG       0x00    //Configuration                     16b
#define INA238_REG_ADC_CONFIG   0x01    //ADC Configuration                 16b
#define INA238_REG_VSHUNT_CAL   0x02    //Shunt calibration                 16b
#define INA238_REG_VSHUNT       0x04    //Shunt Voltage Measurement         16b
#define INA238_REG_VBUS         0x05    //Bus Voltage Measurement           16b
#define INA238_REG_DIETEMP      0x06    //Temperature Measurement           16b
#define INA238_REG_CURRENT      0x07    //Current Result                    16b
#define INA238_REG_POWER        0x08    //Power Result                      24b
#define INA238_REG_DIAG_ALERT   0x0B    //Diagnostic Flags and Alert        16b
#define INA238_REG_SOLV         0x0C    //Shunt Overvoltage Threshold       16b
#define INA238_REG_SULV         0x0D    //Shunt Undervoltage Threshold      16b
#define INA238_REG_BOLV         0x0E    //Bus Overvoltage Threshold         16b
#define INA238_REG_BULV         0x0F    //Bus Undervoltage Threshold        16b
#define INA238_REG_TEMP_LIMIT   0x10    //Temperature Over-Limit Threshold  16b
#define INA238_REG_PWR_LIMIT    0x10    //Power Over-Limit Threshold        16b
#define INA238_REG_MANID        0x3E    //Manufacturer ID                   16b
#define INA238_REG_DEVID        0x3F    //Device ID                         16b



// Bitmask helpers voor DIAG_ALERT register
#define INA238_ALERT_OVERVOLTAGE  (1 << 2)
#define INA238_ALERT_UNDERVOLTAGE (1 << 3)
#define INA238_ALERT_OVERCURRENT  (1 << 4)
#define INA238_ALERT_TEMPERATURE  (1 << 6)
#define INA238_ALERT_CONVERSION   (1 << 7)
#define INA238_ALERT_MEM_ERROR    (1 << 8)

// Universele functies
HAL_StatusTypeDef INA238_WriteRegister(uint8_t reg, uint16_t value);
uint16_t INA238_ReadRegister(uint8_t addr, uint8_t reg);
uint16_t INA238_ReadVbus(uint8_t addr, uint8_t reg);


#endif // INA238_H


