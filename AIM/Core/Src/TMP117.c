/*
 * TMP117.c
 *
 *  Created on: Jun 28, 2025
 *      Author: basge
 */

#include "tmp117.h"
#include <stdio.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

float AlarmLimitHighSetPoint = 0;
float AlarmLimitLowSetPoint  = 0;

/*----------------------------------------------------------------------------*/
/*— I2C TMP117 communicatie functies
----------------------------------------------------------------------------*/

/**
 * Lees een 16-bit register uit de TMP117 via I²C
 * @param addr  7-bit I²C-adres van de TMP117 (linksshifted <<1 voor HAL)
 * @param reg   Registerpointer (8-bit) binnen de TMP117
 * @return      16-bit registerwaarde (MSB<<8 | LSB)
 *
 * Deze functie gebruikt HAL_I2C_Mem_Read om twee bytes te lezen. Bij een I²C-fout
 * wordt een debugbericht via UART verzonden. De read-buf wordt ongeacht de status
 * samengevoegd tot een 16-bit resultaat.
 */
uint16_t TMP117_ReadRegister(uint8_t addr, uint8_t reg) {
    uint8_t rx[2];
    HAL_StatusTypeDef HALStatus;
    // Probeer register reg te lezen, 2 bytes, blocking tot HAL_MAX_DELAY
    HALStatus =HAL_I2C_Mem_Read(&hi2c1,
                         addr,                    // slave-adres inclusief R/W-bit
                         reg,                     // pointer naar het register
                         I2C_MEMADD_SIZE_8BIT,    // registeradresgrootte
                         rx,                      // ontvangbuffer
                         2,                       // aantal te lezen bytes
                         HAL_MAX_DELAY);
    if (HALStatus != HAL_OK){
        // Bij communicatieprobleem: log foutmelding via UART
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "TMP117: Adres 0x%02X, geen I2C-communicatie\r\n",
                           (addr >> 1));  // shift terug naar 7-bit adres voor leesbaarheid
        HAL_UART_Transmit(&huart2,
                          (uint8_t*)buf,
                          len,
                          HAL_MAX_DELAY);
        return 0xffff;
    }else{
        // Combineer MSB en LSB tot één 16-bit waarde en geef terug
    return (uint16_t)((rx[0] << 8) | rx[1]);
    }
}

/**
 * Schrijf een 16-bit waarde (big-endian) naar een register van de TMP117 via I²C
 * @param addr   7-bit I²C-adres van de TMP117 (linksshifted <<1 voor HAL)
 * @param reg    Registerpointer (8-bit) binnen de TMP117
 * @param value  16-bit waarde die we willen schrijven (MSB eerst)
 * @return       HAL-status: HAL_OK bij succes, anders foutcode
 *
 * Deze functie zet de 16-bit waarde om in twee bytes (MSB, LSB) en verstuurt
 * deze met HAL_I2C_Mem_Write. Bij een I²C-fout wordt een debugbericht via UART
 * uitgezonden, waarna de foutstatus wordt teruggegeven.
 */
HAL_StatusTypeDef TMP117_WriteRegister(uint8_t addr, uint8_t reg, uint16_t value) {
    uint8_t tx[2];

    // Splits de 16-bit waarde in twee bytes (big-endian)
    tx[0] = (value >> 8) & 0xFF;  // MSB
    tx[1] =  value        & 0xFF; // LSB

    // Verstuur de twee bytes naar register 'reg'
    HAL_StatusTypeDef HAL_check = HAL_I2C_Mem_Write(
        &hi2c1,                 // I2C-handle
        addr,                   // slave-adres inclusief R/W-bit
        reg,                    // registerpointer
        I2C_MEMADD_SIZE_8BIT,   // registeradresgrootte
        tx,                     // pointer naar transmit-buffer
        2,                      // aantal te schrijven bytes
        HAL_MAX_DELAY           // blocking timeout
    );

    // Bij fout: log melding via UART
    if (HAL_check != HAL_OK) {
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
            "TMP117: Adres 0x%02X, geen I2C-communicatie\r\n",
            (addr >> 1)  // terug naar 7-bit adres voor leesbaarheid
        );
        HAL_UART_Transmit(&huart2,
                          (uint8_t*)buf,
                          len,
                          HAL_MAX_DELAY);
    }

    return HAL_check;
}

/**
 * Lees en converteer de temperatuur van de TMP117.
 *
 * @param addr   7-bit I²C-adres van de TMP117 (wordt linksshifted <<1 voor HAL-functies).
 * @param reg    Registeradres voor het temperatuurresultaat (bijv. TMP117_REG_TEMP_RESULT).
 * @param debug  Zet op 1 om de temperatuurwaarde via UART te versturen; 0 om dit uit te schakelen.
 * @return       Ruwe 16-bit registerwaarde (signed, LSB = 7.8125 m°C).
 *
 * Deze functie:
 * 1. Leest het 16-bit temperatuurregister via TMP117_ReadRegister().
 * 2. Zet de ruwe waarde om naar een signed int16_t.
 * 3. Converteert deze naar graden Celsius (°C = raw * 0.0078125).
 * 4. Stuurt optioneel een debugbericht via UART als 'debug' actief is.
 * 5. Geeft de originele raw-waarde terug voor verder gebruik of logging.
 */

uint16_t TMP117_ReadTemperatureC(uint8_t addr, uint8_t reg, uint8_t debug) {
	// Lees ruwe 16-bit temperatuur
	uint16_t raw = TMP117_ReadRegister(addr, reg);

	// Converteer naar signed waarde
	int16_t signed_val = (int16_t)raw;

	// Bereken temperatuur in graden Celsius
	float temp = signed_val * 0.0078125f;  // 7.8125 m°C per bit

	// Debug-output via UART
	if(debug == 1){
		char buf[64];
		int len = snprintf(buf, sizeof(buf),
					   "TMP117: Adres 0x%02X, temperatuur = %.2f °C\r\n",
					   (addr >> 1), temp);
		HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
	}
	// Return raw registerwaarde (LSB = 7.8125 m°C)
	return raw;
}

/**
 * Schrijf een temperatuur­waarde naar de TMP117
 * @param addr  7-bit I²C-adres van de TMP117 (linksshifted <<1 voor HAL)
 * @param reg   Registerpointer voor het alarm- of T<sub>nA</sub>-register
 * @param temp  Gewenste temperatuur in °C (float)
 * @return      HAL_OK bij succesvolle write, anders foutcode
 *
 * Deze functie:
 * 1. Zet de temperatuur (°C) om naar de ruwe 16-bit weergave
 *    (LSB = 7.8125 m°C) en rond af naar het dichtstbijzijnde geheel getal.
 * 2. Packt de 16-bit waarde als MSB eerst, LSB daarna.
 * 3. Schrijft beide bytes via HAL_I2C_Mem_Write naar het opgegeven register.
 * 4. Bij een I²C-fout wordt een debugbericht via UART uitgezonden.
 */
HAL_StatusTypeDef TMP117_WriteTemperatureC(uint8_t addr, uint8_t reg, float temp) {
    // 1. Converteer °C naar raw ADC-waarde (int16)
    uint16_t raw = (int16_t)lroundf(temp / 0.0078125f);

    // 2. Split raw-waarde in MSB en LSB (big-endian payload)
    uint8_t payload[2];
    payload[0] = (uint8_t)(raw >> 8);   // MSB
    payload[1] = (uint8_t)(raw & 0xFF); // LSB

    // 3. Schrijf 2 bytes naar register 'reg'
    HAL_StatusTypeDef HAL_check = HAL_I2C_Mem_Write(
        &hi2c1,                 // I²C-handle
        addr,                   // slave-adres inclusief R/W-bit
        reg,                    // registerpointer
        I2C_MEMADD_SIZE_8BIT,   // registeradresgrootte
        payload,                // pointer naar payload-buffer
        sizeof(payload),        // aantal bytes
        HAL_MAX_DELAY           // timeout
    );

    // 4. Bij fout: log via UART
    if (HAL_check != HAL_OK) {
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
            "TMP117: Adres 0x%02X, geen I2C-communicatie\r\n",
            (addr >> 1) // 7-bit adres voor leesbaarheid
        );
        HAL_UART_Transmit(&huart2,
                          (uint8_t*)buf,
                          len,
                          HAL_MAX_DELAY);
    }

    return HAL_check;
}

/**
 * @brief  Initialiseert de TMP117 met configuratie en optionele temperatuurgrenzen.
 *         Schrijft in één keer alle opgegeven instellingen.
 * @param  addr          I²C slave-adres van de TMP117 (8-bit)
 * @param  config        Waarde voor het CONFIG-register (bijv. continuous mode, averaging)
 * @param  temp_high     Bovenste temperatuurgrens (°C)
 * @param  temp_low      Onderste temperatuurgrens (°C)
 */
void TMP117_Init(uint8_t addr, uint16_t config, float temp_high, float temp_low)
{
    const float lsb = 0.0078125f;
    int16_t raw_high = (int16_t)lroundf(temp_high / lsb);
    int16_t raw_low  = (int16_t)lroundf(temp_low  / lsb);
    char buf[80];

    // 1) Shutdown‐mode: MODE = 00 → clear bits[10:9]
    uint16_t cfg_shutdown = config & ~(0x0300);
    TMP117_WriteRegister(addr, TMP117_REG_CONFIG, cfg_shutdown);
    HAL_Delay(100);

    // 2) Write thresholds while in shutdown
    TMP117_WriteRegister(addr, TMP117_REG_T_HIGH_LIMIT, (uint16_t)raw_high);
    HAL_Delay(100);
    uint16_t v = TMP117_ReadRegister(addr, TMP117_REG_T_HIGH_LIMIT);
    snprintf(buf, sizeof(buf),
        "DBG: T_HIGH write: exp=0x%04X, rd=0x%04X\r\n", raw_high, v);
    HAL_UART_Transmit(&huart2,(uint8_t*)buf,strlen(buf),HAL_MAX_DELAY);

    TMP117_WriteRegister(addr, TMP117_REG_T_LOW_LIMIT, (uint16_t)raw_low);
    HAL_Delay(100);
    v = TMP117_ReadRegister(addr, TMP117_REG_T_LOW_LIMIT);
    snprintf(buf, sizeof(buf),
        "DBG: T_LOW  write: exp=0x%04X, rd=0x%04X\r\n", raw_low, v);
    HAL_UART_Transmit(&huart2,(uint8_t*)buf,strlen(buf),HAL_MAX_DELAY);

    // 3) Continuous‐mode: schrijf CONFIG met MODE=11 (bits10:9)
    TMP117_WriteRegister(addr, TMP117_REG_CONFIG, config);
    HAL_Delay(100);

    // 4) Debug final CONFIG
    v = TMP117_ReadRegister(addr, TMP117_REG_CONFIG);
    snprintf(buf, sizeof(buf),
        "DBG: CONFIG final: rd=0x%04X\r\n", v);
    HAL_UART_Transmit(&huart2,(uint8_t*)buf,strlen(buf),HAL_MAX_DELAY);
}



/**
 * Toon de inhoud van een 16-bit register van de TMP117 via UART
 * @param addr      7-bit I²C-adres van de TMP117 (linksshifted <<1 voor HAL)
 * @param Register  Registerpointer dat gelezen wordt (bijv. TMP117_REG_CONFIG)
 *
 * Deze functie:
 * 1. Leest het opgegeven register via TMP117_ReadRegister().
 * 2. Zet de 16-bit waarde om naar een binaire string (MSB eerst).
 * 3. Stuurt de binaire en hexadecimale representatie via UART.
 */
void TMP117_Display_Register(uint8_t addr, uint16_t Register)
{
    // Lees de 16-bit registerwaarde
    uint16_t ConfigRegister = TMP117_ReadRegister(addr, Register);
    if(ConfigRegister != 0xffff){
    // Buffer voor binaire representatie (16 tekens + terminator)
    char bufbin[17];
    for (int i = 0; i < 16; i++) {
        // Controleer bit (15 - i) en zet '1' of '0'
        bufbin[i] = (ConfigRegister & (1 << (15 - i))) ? '1' : '0';
    }
    bufbin[16] = '\0';  // String-terminator

    // Maak een UART-outputstring met binaire en hex weergave
    char buf[128];
   	int len = snprintf(buf, sizeof(buf),
                       "TMP117:  Adres 0x%02X, Read Reg op: 0x%04X = 0b%s (0x%04X) \r\n",
                       (addr >> 1),       // shift terug naar 7-bit adres
					   (unsigned int)Register,
					   bufbin,            // binaire weergave
                       (unsigned int)ConfigRegister); // hex weergave
    	HAL_UART_Transmit(&huart2,(uint8_t*)buf,len, HAL_MAX_DELAY);
    }
}

void TMP117_Alert(uint16_t addr){
	uint16_t rawtemp = TMP117_ReadRegister(addr, TMP117_REG_TEMP_RESULT);

	int16_t signed_val = (int16_t)rawtemp;
	float temp = signed_val * 0.0078125f;

	uint16_t raw_lim_high = TMP117_ReadRegister(addr, TMP117_REG_T_HIGH_LIMIT);

	int16_t signed_raw_lim_high = (int16_t)raw_lim_high;
	float lim_high = signed_raw_lim_high * 0.0078125f;

	uint16_t raw_lim_low = TMP117_ReadRegister(addr, TMP117_REG_T_LOW_LIMIT);

	int16_t signed_raw_lim_low = (int16_t)raw_lim_low;
	float lim_low = signed_raw_lim_low * 0.0078125f;

	char buf[80];
	int len = snprintf(buf, sizeof(buf),
				  "Alarm! T=%.2f °C  limieten: low=%.2f, high=%.2f\r\n",
				  temp, lim_low, lim_high);
	HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

void TMP117_DebugShowLimits(uint8_t addr)
{
	uint16_t raw_high = TMP117_ReadRegister(addr, TMP117_REG_T_HIGH_LIMIT);
	uint16_t raw_low  = TMP117_ReadRegister(addr, TMP117_REG_T_LOW_LIMIT);

	float high = ((int16_t)raw_high) * 0.0078125f;
	float low  = ((int16_t)raw_low)  * 0.0078125f;

	char buf[80];
	int len = snprintf(buf, sizeof(buf),
		"T_HIGH = %.2f °C, T_LOW = %.2f °C\r\n", high, low);
	HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

