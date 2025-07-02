/*
 * INA238.c
 *
 *  Created on: Jun 28, 2025
 *      Author: basge
 */


#include "INA238.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart2;


/*----------------------------------------------------------------------------*/
/*— I2C INA238 sensor communicatie functies
/*----------------------------------------------------------------------------*/

/**
 * @brief  Leest een 16-bit register van de INA238 via I²C.
 * @param  addr   8-bit I²C-adres van de INA238 (linksshifted voor HAL).
 * @param  reg    Registeradres binnen de INA238.
 * @retval 16-bit registerwaarde, of 0xFFFF bij fout.
 */
uint16_t INA238_ReadRegister(uint8_t addr, uint8_t reg)
{
    uint8_t rx[2];
    HAL_StatusTypeDef HALStatus;

    // Lees 2 bytes uit het opgegeven register (blokkerend)
    HALStatus = HAL_I2C_Mem_Read(&hi2c3,          // I²C-handle voor INA238
                                 addr,            // 8-bit slave-adres
                                 reg,             // Registeradres
                                 I2C_MEMADD_SIZE_8BIT,
                                 rx,              // Buffer om 2 bytes in te lezen
                                 2,
                                 HAL_MAX_DELAY);  // Blokkerende wachttijd

    if (HALStatus != HAL_OK) {
        // Bij fout: stuur foutmelding via UART
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
            "INA238: Adres 0x%02X, geen I2C-communicatie\r\n",
            (addr >> 1));  // Terug naar 7-bit voor leesbaarheid

        HAL_UART_Transmit(&huart2,
                          (uint8_t*)buf,
                          len,
                          HAL_MAX_DELAY);
        return 0xFFFF;  // Foutwaarde
    } else {
        // Combineer MSB en LSB tot één 16-bit waarde
        return (uint16_t)((rx[0] << 8) | rx[1]);
    }
}


/**
 * @brief  Toon de inhoud van een 16-bit register van de INA238 via UART.
 * @param  addr      8-bit I²C-adres van de INA238 (linksshifted voor HAL).
 * @param  reg       Registeradres binnen de INA238 (bijv. INA238_REG_CONFIG).
 *
 * Deze functie:
 * 1. Leest het opgegeven register via INA238_ReadRegister().
 * 2. Zet de 16-bit waarde om naar een binaire string (MSB eerst).
 * 3. Stuurt de binaire en hexadecimale representatie via UART.
 */
void INA238_Display_Register(uint8_t addr, uint8_t reg)
{
    // Lees de 16-bit registerwaarde
    uint16_t reg_val = INA238_ReadRegister(addr, reg);
    if (reg_val != 0xFFFF)  // 0xFFFF bij fout, aanpasbaar naar je eigen error handling
    {
        // Buffer voor binaire representatie (16 bits + null-terminator)
        char bin_str[17];
        for (int i = 0; i < 16; i++) {
            bin_str[i] = (reg_val & (1 << (15 - i))) ? '1' : '0';
        }
        bin_str[16] = '\0';

        // Format output (binaire + hex + registeradres)
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
            "INA238: Adres 0x%02X, Reg 0x%02X = 0b%s (0x%04X)\r\n",
            (addr >> 1),  // 7-bit adres voor leesbaarheid
            reg,
            bin_str,
            (unsigned int)reg_val);

        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }
}

/**
 * @brief  Schrijft een 16-bit waarde naar een register van de INA238 via I²C.
 * @param  addr: Het 8-bit I²C slave-adres van de INA238 (inclusief R/W-bit op 0).
 * @param  reg:  Het registeradres binnen de INA238 (8-bit).
 * @param  value: De 16-bit waarde die naar het register geschreven moet worden.
 * @retval HAL status (HAL_OK bij succes, anders foutcode).
 */
HAL_StatusTypeDef INA238_WriteRegister(uint8_t addr, uint8_t reg, uint16_t value)
{
    uint8_t tx[2];

    // Stap 1: Splits de 16-bit waarde op in 2 bytes (big-endian formaat)
    // MSB eerst (INA238 verwacht data in big-endian volgorde)
    tx[0] = (value >> 8) & 0xFF;  // Most Significant Byte
    tx[1] =  value        & 0xFF; // Least Significant Byte

    // Stap 2: Verstuur de twee bytes naar het opgegeven register via I²C
    HAL_StatusTypeDef HAL_check = HAL_I2C_Mem_Write(
        &hi2c3,                // I²C interface gebruikt voor de INA238
        addr,                  // Slave-adres van de INA238 (meestal 0x80 t/m 0x8E)
        reg,                   // Adres van het register binnen de INA238
        I2C_MEMADD_SIZE_8BIT,  // Adresgrootte van het register (8-bit)
        tx,                    // Buffer met de te verzenden bytes (MSB, LSB)
        2,                     // Aantal bytes dat verstuurd wordt
        HAL_MAX_DELAY          // Maximale wachttijd (blokkerend)
    );

    // Stap 3: Foutafhandeling (optioneel)
    // Als de transmissie is mislukt, stuur dan een foutmelding via UART voor debugdoeleinden
    if (HAL_check != HAL_OK)
    {
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
            "INA238: Adres 0x%02X, geen I2C-communicatie\r\n",
            (addr >> 1)  // Converteer naar 7-bit adres voor leesbaarheid
        );
        HAL_UART_Transmit(&huart2,
                          (uint8_t*)buf,
                          len,
                          HAL_MAX_DELAY);
    }

    // Stap 4: Retourneer de status van de I²C-operatie
    return HAL_check;
}

/**
 * @brief  Initialiseert de INA238 met vooraf gedefinieerde configuratie- en drempelwaarden.
 *         Schrijft alle relevante instellingen in één keer naar het apparaat.
 * @param  addr          I²C slave-adres van de INA238 (8-bit)
 * @param  config        Waarde voor het CONFIG-register (bijv. mode & averaging)
 * @param  adc_config    Waarde voor het ADC_CONFIG-register (meetsnelheid, mode)
 * @param  shunt_cal     Kalibratiewaarde voor shuntmeting (bijv. om stroom in A te krijgen)
 * @param  diag_alert    Waarde voor het DIAG_ALERT-register (welke drempels triggeren alerts)
 * @param  solv_threshold Spanning (in volt) voor Shunt Over Limit drempel
 * @param  sulv_threshold Spanning voor Shunt Under Limit drempel
 * @param  bovl_threshold Spanning voor Bus Over Voltage Limit drempel
 * @param  buvl_threshold Spanning voor Bus Under Voltage Limit drempel
 */
void INA238_Init(uint8_t addr,
                 uint16_t shunt_cal,
                 uint16_t diag_alert,
                 float sovl_threshold,
                 float suvl_threshold,
                 float bovl_threshold,
                 float buvl_threshold)
{
    float lsb_voltage = 0.003125f;

    // Zet spanningsdrempels om naar raw registerwaarden
    uint16_t sovl_raw = ConvertVoltageToRaw(sovl_threshold, lsb_voltage);
    uint16_t suvl_raw = ConvertVoltageToRaw(suvl_threshold, lsb_voltage);
    uint16_t bovl_raw = ConvertVoltageToRaw(bovl_threshold, lsb_voltage);
    uint16_t buvl_raw = ConvertVoltageToRaw(buvl_threshold, lsb_voltage);

    char buf[64];

    // Stap 1: Set ADC_CONFIG (optioneel, maar expliciet is veilig)
    INA238_WriteRegister(addr, INA238_REG_ADC_CONFIG, 0xFB68);
    HAL_Delay(2);

    // Stap 2: Calibratie en drempels
    INA238_WriteRegister(addr, INA238_REG_VSHUNT_CAL, shunt_cal);
    INA238_WriteRegister(addr, INA238_REG_SOLV, sovl_raw);
    INA238_WriteRegister(addr, INA238_REG_SULV, suvl_raw);
    INA238_WriteRegister(addr, INA238_REG_BOLV, bovl_raw);
    INA238_WriteRegister(addr, INA238_REG_BULV, buvl_raw);
    HAL_Delay(2);

    // Stap 3: Zet DIAG_ALERT met ALERT_EN (bit 14)
    INA238_WriteRegister(addr, INA238_REG_DIAG_ALERT, diag_alert);
    HAL_Delay(2);
}



/**
 * Lees de VBUS-spanning van de INA238 en converteer naar volts
 * @param addr  7-bit I²C-adres van de INA238 (linksshifted <<1 voor HAL)
 * @param reg   Registerpointer voor het VBUS-register
 * @return      Raw 16-bit registerwaarde (LSB = 3.125 mV)
 *
 * Deze functie:
 * 1. Roept INA238_ReadRegister() aan om de raw data in te lezen.
 * 2. Converteert de signed 16-bit waarde naar spanning in volt.
 * 3. Stuurt een debugbericht via UART met de gemeten spanning.
 * 4. Geeft de raw registerwaarde terug voor eventueel verder gebruik.
 */
uint16_t INA238_ReadVoltage(uint8_t addr, uint8_t reg, uint8_t debug) {
    // 1. Lees de raw 16-bit waarde
    uint16_t raw = INA238_ReadRegister(addr, reg);

    // 2. Converteer naar signed en bereken spanning (3.125 mV per bit)
    int16_t signed_val = (int16_t)raw;
    float voltage = signed_val * 3.125e-3f;  // V

    // 3. Debug-output via UART
    if(debug == 1){
    	char buf[64];
    	int len = snprintf(buf, sizeof(buf),
                       "INA238: Adres 0x%02X: Voltage = %.2f V\r\n",
                       addr, voltage);
    	HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }
    // 4. Return raw registerwaarde
    return raw;
}

/**
 * @brief  Leest de stroomwaarde uit een INA238-register en stuurt optioneel debuginfo via UART.
 *
 * @param  addr   Het 7-bit I2C-adres van de INA238 (linker-shifted indien nodig).
 * @param  reg    Het registeradres waaruit de stroom gemeten wordt (INA238_REG_CURRENT).
 * @param  debug  Indien 1: verstuur debugoutput via UART.
 *
 * @retval raw  De ruwe 16-bit registerwaarde (signed), zoals uit het device gelezen.
 */
uint16_t INA238_ReadCurrent(uint8_t addr, uint8_t reg, uint8_t debug)
{
    // 1) Lees 16-bit waarde uit opgegeven register
    uint16_t raw = INA238_ReadRegister(addr, reg);

    // 2) Zet om naar signed waarde (INA238 levert signed current in 2's complement)
    int16_t signed_val = (int16_t)raw;

    // 3) Reken ruwe waarde om naar ampère (LSB = 0.5 mA = 5e-4 A volgens datasheet)
    float current = signed_val * 5e-4f;

    // 4) Optioneel: stuur de stroomwaarde via UART (voor debugdoeleinden)
    if (debug == 1) {
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "INA238: Adres 0x%02X: Current = %.3f A\r\n",
                           addr, current);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

    // 5) Geef de ruwe waarde terug
    return raw;
}

/**
 * @brief  Leest de 24-bit vermogenswaarde uit de INA238 en stuurt optioneel debugoutput via UART.
 *
 * @param  addr   Het 7-bit I2C-adres van de INA238 (left-shifted bij gebruik in HAL).
 * @param  reg    Het registeradres (moet INA238_REG_POWER zijn, = 0x03).
 * @param  debug  Indien 1: toon vermogenswaarde via UART.
 *
 * @retval raw24  De ruwe 24-bit waarde (in een 32-bit container).
 */
uint32_t INA238_ReadPower(uint8_t addr, uint8_t reg, uint8_t debug)
{
    uint8_t rx[3];
    HAL_StatusTypeDef status;

    // Lees 3 bytes (24 bits) uit het vermogensregister
    status = HAL_I2C_Mem_Read(&hi2c3,
                              addr,                 // 8-bit I²C-adres
                              reg,                  // Registeradres (0x03)
                              I2C_MEMADD_SIZE_8BIT,
                              rx,                   // Buffer voor 3 bytes
                              3,                    // Aantal te lezen bytes
                              HAL_MAX_DELAY);

    if (status != HAL_OK) {
        // Debug: fout bij communicatie
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "INA238: Adres 0x%02X, geen I2C-communicatie\r\n",
                           (addr >> 1));
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
        return 0xFFFFFF;  // Foutwaarde
    }

    // Combineer de 3 bytes tot één 24-bit waarde (MSB first)
    uint32_t raw = ((uint32_t)rx[0] << 16) |
                   ((uint32_t)rx[1] << 8)  |
                   (uint32_t)rx[2];

    // Vermogen berekenen in watt (volgens datasheet):
    // Power_LSB = Current_LSB * 25, aannemend Current_LSB = 0.0005 A
    float power = raw * 0.0005f * 25.0f;  // = raw * 0.0125f

    if (debug) {
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "INA238: Adres 0x%02X: Power = %.3f W (raw=0x%06lX)\r\n",
                           (addr >> 1), power, (unsigned long)raw);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

    return raw;
}
/**
 * @brief  Leest de interne temperatuur van de INA238 via I2C en toont optioneel debug-informatie via UART.
 * @param  addr: I2C-adres van de INA238 (8-bit, dus inclusief R/W-bit op 0)
 * @param  debug: indien 1, wordt de temperatuur via UART weergegeven
 * @retval ruwe (geshiftte) temperatuurwaarde zoals geleverd door de sensor
 *
 * Deze functie is bedoeld voor STM32-microcontrollers met HAL. De I2C-verbinding wordt verondersteld via 'hi2c3'.
 * De temperatuur wordt uit het DIETEMP-register gelezen, dat 13-bit gesigneerde data bevat (uitgelijnd op bits 15:3).
 */
uint16_t INA238_ReadTemp(uint8_t addr, uint8_t debug){
    // 1. Lees 16-bit ruwe waarde uit het DIETEMP-register (adres 0x0C)
    uint16_t raw = INA238_ReadRegister(addr, INA238_REG_DIETEMP);

    // 2. De onderste 4 bits zijn altijd nul, dus we verschuiven 4 bits naar rechts
    raw >>= 4;

    // 3. Tekenextensie indien het hoogste bit (bit 12 van 13-bit waarde) gezet is
    //    Hierdoor wordt de 13-bit waarde correct gesigneerd wanneer deze gecast wordt
    if (raw & 0x1000) {
        raw |= 0xE000;  // Zet de bovenste bits voor negatieve waarden
    }
    int16_t signed_val = (int16_t)raw;

    // 4. Reken om naar graden Celsius (1 LSB = 0.125 °C volgens datasheet)
    float temp = signed_val * 0.125f;

    // 5. Optioneel: stuur debug-informatie via UART2
    if (debug == 1){
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "INA238: Adres 0x%02X, temperatuur = %.2f °C\r\n",
                           (addr >> 1), temp);  // addr>>1 om 7-bit slave-adres weer te geven
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

    // 6. Geef de ruwe waarde (13-bit gesigneerde int) terug voor eventueel CAN-gebruik
    return raw;
}


void INA238_Alert(uint16_t addr)
{
    uint16_t status = INA238_ReadRegister(addr, INA238_REG_DIAG_ALERT);  // 0x0C
    INA238_Display_Register(addr, INA238_REG_DIAG_ALERT);

    if (status == 0) {
        return; // Geen actieve alerts
    }

    if (status & INA238_ALERT_MEM_ERROR) {
        // geheugenfout
        // InaMemError();
    }
    if (status & INA238_ALERT_TEMPERATURE) {
        // temperatuurdrempel overschreden
        // InaTempOver();
    }
    if (status & INA238_ALERT_SHUNT_OVER) {
        // shuntspanning te hoog
        // InaShuntOver();
    }
    if (status & INA238_ALERT_SHUNT_UNDER) {
        // shuntspanning te laag
        // InaShuntUnder();
    }
    if (status & INA238_ALERT_BUS_OVERVOLTAGE) {
        // VBUS te hoog
        // InaBusOver();
    }
    if (status & INA238_ALERT_BUS_UNDERVOLTAGE) {
        // VBUS te laag
        // InaBusUnder();
    }
    if (status & INA238_ALERT_OVERCURRENT) {
        // stroom overschreden
        // InaCurrentOver();
    }
    if (status & INA238_ALERT_CONVERSION) {
        // conversie voltooid
        // InaConversionDone();
    }
}

/*
			  uint16_t rawvoltage = INA238_ReadVoltage(INA238_I2C_ADDR,INA238_REG_VBUS,0);
			  uint16_t rawcurrent = INA238_ReadCurrent(INA238_I2C_ADDR, INA238_REG_CURRENT, 0);

			  int16_t signed_val_V = (int16_t)rawvoltage;
			  float voltage = signed_val * 3.125e-3f;  // V

			  int16_t signed_val_I = (int16_t)rawcurrent;
			  float current = signed_val * 3.125e-3f;  // V
			  char buf[80];
			  int len = snprintf(buf, sizeof(buf),
					  "Alarm! Spanning=%.2f, Stroom=%.2f A  limieten: Spanning low=%.2f, high=%.2f stroom low=%.2f, high=%.2f\r\n",
							  voltage, current, AlarmLimitLowSetPoint, AlarmLimitHighSetPoint)
			  // Lees
		  }
}
*/
