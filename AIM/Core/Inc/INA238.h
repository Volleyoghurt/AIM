/*
 * INA238.h
 *
 *  Created on: Jun 13, 2025
 *      Author: basge
 */

#include "stm32l4xx_hal.h"

#ifndef INC_INA238_H_
#define INC_INA238_H_

// I2C-adres van de INA238 (7-bit adres links verschoven voor HAL)
#define INA238_I2C_ADDR   (0x40 << 1)

// Registeradressen
#define INA238_REG_CONFIG       0x00    //Configuration                     16b    	rw
#define INA238_REG_ADC_CONFIG   0x01    //ADC Configuration                 16b		rw
#define INA238_REG_VSHUNT_CAL   0x02    //Shunt calibration                 16b		rw
#define INA238_REG_VSHUNT       0x04    //Shunt Voltage Measurement         16b		R		Conversion factor: 5 μV/LSB when ADCRANGE = 0 1.25 μV/LSB when ADCRANGE = 1
#define INA238_REG_VBUS         0x05    //Bus Voltage Measurement           16b		R		Conversion factor: 3.125 mV/LSB
#define INA238_REG_DIETEMP      0x06    //Temperature Measurement           16b		R		Conversion factor: 125 m°C/LSB
#define INA238_REG_CURRENT      0x07    //Current Result                    16b		R     	Conversion Factor: 5 μV/LSB when ADCRANGE = 0 1.25 μV/LSB when ADCRANGE = 1.
#define INA238_REG_POWER        0x08    //Power Result                      24b		R
#define INA238_REG_DIAG_ALERT   0x0B    //Diagnostic Flags and Alert        16b		rw
#define INA238_REG_SOLV         0x0C    //Shunt Overvoltage Threshold       16b		RW		Conversion Factor: 5 μV/LSB when ADCRANGE = 0 1.25 μV/LSB when ADCRANGE = 1
#define INA238_REG_SULV         0x0D    //Shunt Undervoltage Threshold      16b		rw		Conversion Factor: 5 μV/LSB when ADCRANGE = 0 1.25 μV/LSB when ADCRANGE = 1
#define INA238_REG_BOLV         0x0E    //Bus Overvoltage Threshold         16b		rw		Unsigned representation, positive value only. Conversion factor: 3.125 mV/LSB.
#define INA238_REG_BULV         0x0F    //Bus Undervoltage Threshold        16b		rw		Unsigned representation, positive value only. Conversion factor: 3.125 mV/LSB.
#define INA238_REG_TEMP_LIMIT   0x10    //Temperature Over-Limit Threshold  16b		rw		Conversion factor: 125 m°C/LSB.
#define INA238_REG_PWR_LIMIT    0x10    //Power Over-Limit Threshold        16b		rw		Conversion factor: 256 × Power LSB
#define INA238_REG_MANID        0x3E    //Manufacturer ID                   16b		r
#define INA238_REG_DEVID        0x3F    //Device ID                         16b		r


#define INA238_ALERT_SHUNT_OVER      (1 << 0)
#define INA238_ALERT_SHUNT_UNDER     (1 << 1)
#define INA238_ALERT_BUS_OVERVOLTAGE (1 << 2)
#define INA238_ALERT_BUS_UNDERVOLTAGE (1 << 3)
#define INA238_ALERT_OVERCURRENT     (1 << 4)
#define INA238_ALERT_CONVERSION      (1 << 5)
#define INA238_ALERT_TEMPERATURE     (1 << 6)
#define INA238_ALERT_CONV_READY      (1 << 7)
#define INA238_ALERT_MEM_ERROR       (1 << 8)


#define INA238_ALERT_ENABLE_PIN      (1 << 14)

#define INA238_ALERT_ALL_WITH_ENABLE  (0x4000 | 0x01FF)  // bit 14 + alle alerts

typedef struct {
    uint8_t reg;
    uint16_t mask;
    uint16_t value;
} INA238_WriteConfig_t;


// Universele functies



void INA238_Init(uint8_t addr,
                 uint16_t shunt_cal,
                 uint16_t diag_alert,
                 float sovl_threshold,
                 float suvl_threshold,
                 float bovl_threshold,
                 float buvl_threshold);
HAL_StatusTypeDef INA238_WriteRegister(uint8_t addr, uint8_t reg, uint16_t value);
HAL_StatusTypeDef INA238_WriteVoltageRegister(uint8_t addr, uint8_t reg, float voltage);
uint16_t INA238_ReadRegister(uint8_t addr, uint8_t reg);
uint16_t INA238_ReadVoltage(uint8_t addr, uint8_t reg,uint8_t debug);
uint16_t INA238_ReadCurrent(uint8_t addr, uint8_t reg, uint8_t debug);
uint16_t INA238_ReadTemp(uint8_t addr, uint8_t debug);
uint32_t INA238_ReadPower(uint8_t addr, uint8_t reg, uint8_t debug);
uint16_t ConvertVoltageToRaw(float voltage, float lsb);
void INA238_Alert(uint16_t addr);


#endif // INA238_H


