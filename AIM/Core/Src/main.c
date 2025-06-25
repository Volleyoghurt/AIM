/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "can.h"
#include "tmp117.h"
#include "ds1682.h"
#include "INA238.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VERSION 01
#define BUILDDATE 060625
#define SERIAL 1337
#define BOARD 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
void UART_MSG(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeaderCan;
CAN_RxHeaderTypeDef RxHeaderCan;

uint32_t TxDataCan[8];
uint8_t RxDataCan[8];
uint8_t RX_Buffer_I2C[1]; //I2C RX variable
char buf[64];
uint8_t TempReady 		= 0;
uint8_t TmpAlertFlag 	= 0;
uint8_t InaAlertFlag 	= 0;
char TempBuf[64];




uint32_t pulseStart = 0;
uint32_t pulseWidth = 0;

/* Settings:--------------------------------------*/
float AlarmLimitHighSetPoint 	= 27.0f;
float AlarmLimitLowSetPoint 	= 26.0f;

/*************************************************************************/
/* RX gedeelte  */
/*************************************************************************/

// Ontvangst interupt voor CAN-bus berichten (gebruikt voor debug).
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	 CAN_Msg_t msg;
	    // haal header + data op
	 HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msg.header, msg.data);
	    // probeer in queue te zetten (of negeren als vol)
	 EnqueueCAN(&msg.header, msg.data);
}
//Interupt test

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LinkStatus1_Pin)  // Pas aan op jouw pin
    {
        // Lees de actuele stand van de pin
        GPIO_PinState state = HAL_GPIO_ReadPin(LinkStatus1_GPIO_Port, LinkStatus1_Pin);


        // Zet de LED gelijk aan de pulsstatus
       HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, state);

    }

    if (GPIO_Pin == TMPALERT_Pin)
      {
    	  TmpAlertFlag = 1;

     } else {
          __NOP();
    }

    if(GPIO_Pin == InaAlertFlag)
    {
    	InaAlertFlag = 1;
    }
}


// init van de AIM bij eerste opstart MCU

void   AIM_INIT(void)
{
	//Start bericht
	  UART_MSG();
//
	// Temperatuur alarm instellen
	//if(TMP117_ReadRegister(TMP117_I2C_ADDR,TMP117_REG_CONFIG) != 0xffff)
	//{
		TMP_SetAlarmTemp(TMP117_I2C_ADDR, TMP117_REG_T_LOW_LIMIT,AlarmLimitLowSetPoint);
		TMP_SetAlarmTemp(TMP117_I2C_ADDR, TMP117_REG_T_HIGH_LIMIT,AlarmLimitHighSetPoint);
		uint16_t AlarmLimitLow = TMP117_ReadTemperatureC(TMP117_I2C_ADDR, TMP117_REG_T_LOW_LIMIT,0);
		uint16_t AlarmLimitHigh = TMP117_ReadTemperatureC(TMP117_I2C_ADDR, TMP117_REG_T_HIGH_LIMIT,0);


		// TnA config instellen Temp Mode = 1, Alert mode = 0
		TMP117_WriteMaskedRegister(TMP117_I2C_ADDR, TMP117_REG_CONFIG, (1 << 4), (1 << 4));


		TMP117_Display_Register (TMP117_I2C_ADDR, TMP117_REG_CONFIG);
		//}

		INA238_SetAlarmBOVL(INA238_I2C_ADDR,3);


	 	 // Send identificatie van het board
	 	 CanSendIdent(BOARD, 1, VERSION, SERIAL, BUILDDATE);

}


void UART_MSG(void)
{
	char buf[80];
	int len = snprintf(buf, sizeof(buf),"*************AIM gestart!**********\n\r");
  	HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

/*----------------------------------------------------------------------------*/
/*— Can-bus zend-functies  —*/
/*----------------------------------------------------------------------------*/

/**
 * Verstuur een temperatuur-bericht over CAN
 * @param board       Board-/slot-ID (0–63) waaruit het bericht afkomstig is
 * @param index       Vrij te gebruiken index, bijvoorbeeld sensornummer op het board
 * @param temperature Raw temperatuurwaarde (16-bit, LSB = 7.8125 m°C)
 */
void CanSendTemperature(uint8_t board, uint8_t index, uint16_t temperature) {
    // Vul de payloadstructuur met de rauwe temperatuurdata
    TempMsg_t msgtmp;
    msgtmp.value     = temperature;  // byte 0–1: temperatuur in stappen van 7.8125 m°C
    msgtmp.status    = 0x00;         // byte 2: sensorstatus (0 = OK)
    msgtmp.timestamp = 0x1337;       // byte 3–6: tijdstempel (hier voorbeeldwaarde)

    // Bouw de 29-bit Extended CAN-ID:
    // - PRIO_TEMP:   prioriteit voor temperatuur­berichten
    // - board:       board-/slot-ID
    // - TYPE_TEMP:   type-ID voor TempMsg_t
    // - index:       index voor onderscheid tussen meerdere temperatuur­sensoren
    uint32_t idTmp = CAN_EXT_ID(
        PRIO_TEMP,
        board,
        TYPE_TEMP,
        index
    );

    // Verstuur het bericht via CanSendMessage:
    // - idTmp:   de samengestelde Extended CAN-ID
    // - payload: pointer naar onze TempMsg_t struct
    // - sizeof:  payloadgrootte (hier 5 bytes, rest wordt automatisch opgevuld met 0)
    CanSendMessage(idTmp, (uint8_t*)&msgtmp, sizeof(msgtmp),0);
}

/**
 * Stuur een Identificatiebericht over CAN
 * @param board   Board-/slot-ID (0–63) waaruit het bericht afkomstig is
 * @param index   Vrij te gebruiken index, meestal 0 bij Ident-berichten
 * @param id      Firmware- of configuratieversie
 * @param serid   Uniek serienummer van de LRU
 * @param bdate   Bouwdatum van de firmware (bijv. YYMMDD of index)
 */

void CanSendIdent(uint8_t board, uint8_t index, uint8_t id, uint8_t serid, uint8_t bdate){
	 // Maak een IdentMsg_t-struct en vul de velden
		IdentMsg_t msgversion;
		msgversion.version_id = id;
		msgversion.serial_id = serid;
		msgversion.build_date = bdate;

		// Bouw de 29-bit Extended CAN-ID:
		// - PRIO_IDENT: prioriteit voor identificatie­berichten
		// - board:      board-/slot­ID (fysieke plaats van de LRU)
		// - TYPE_IDENT: type-ID voor IdentMsg_t
		// - index:      extra indexveld (kan 0 zijn)
	uint32_t idId = CAN_EXT_ID(
			PRIO_IDENT, // prioriteit
	        board,
			TYPE_IDENT,                // type ID temperatuur
	        index
		    );
		// Verstuur het bericht via CanSendMessage:
		// - idId:     uitgebreide CAN-ID
	    // - payload:  pointer naar onze IdentMsg_t
	    // - length:   grootte van de payload (hier 3 bytes)
	CanSendMessage(idId, (uint8_t*)&msgversion,sizeof(msgversion),0);
}


/**
 * Verstuur een runtime-bericht over CAN
 * @param board           Board-/slot-ID (0–63) waaruit het bericht afkomstig is
 * @param index           Vrij te gebruiken index, bijvoorbeeld LRU-index op het board
 * @param uptime_seconds  Totale uptime in seconden
 * @param reset           Aantal keren dat de MCU is gereset
 */
void CanSendUptime(uint8_t board, uint8_t index, uint32_t uptime_seconds, uint32_t reset) {
    // Vul een RuntimeMsg_t struct met de meegegeven waarden
    RuntimeMsg_t msg;
    msg.runtime     = uptime_seconds;  // byte 0–3: uptime in seconden
    msg.reset_count = reset;           // byte 4: aantal resets (payload wordt automatisch 0-gepad tot 8 bytes)

    // Bouw de 29-bit Extended CAN-ID:
    //  - PRIO_RUNTIME: prioriteit voor uptime-berichten
    //  - board:        board-/slot-ID zodat de master weet uit welk slot dit komt
    //  - TYPE_RUNTIME: type-ID voor runtime-berichten
    //  - index:        extra indexveld ter differentiatie
    uint32_t id = CAN_EXT_ID(
        PRIO_RUNTIME,
        board,
        TYPE_RUNTIME,
        index
    );

    // Verstuur het bericht via de generieke CAN-send functie
    //  - id:      de samengestelde Extended CAN-ID
    //  - payload: pointer naar de RuntimeMsg_t struct
    //  - sizeof:  aantal bytes in de struct (hier 5 bytes)
    CanSendMessage(id, (uint8_t*)&msg, sizeof(msg),0);
}

void CanSendVoltage(uint8_t board, uint8_t index, uint32_t voltage, uint32_t time) {
	VoltageMsg_t msg;
	msg.value = voltage;
	msg.status = 0x00;
	msg.timestamp = time;

	uint32_t id = CAN_EXT_ID(
		PRIO_SPANNING,
		board,
		TYPE_SPANNING,
		index
	);
	CanSendMessage(id, (uint8_t*)&msg, sizeof(msg),0);
}

// Can-bus zend bericht.
/**
 * Verstuur een CAN-bericht via één van de TX-mailboxen van bxCAN
 * @param extId   29-bit Extended Identifier
 * @param payload Pointer naar maximaal 8 bytes data
 * @param length  Aantal bytes in payload (0–8)
 */
void CanSendMessage(uint32_t extId, const uint8_t payload[8], uint8_t length,uint8_t debug)
{
    uint32_t TxMailbox;

    // Controleer of de payload niet langer is dan 8 bytes
    if (length <= 8) {
        // Stel de header in voor een Extended Data Frame
        TxHeaderCan.IDE = CAN_ID_EXT;       // Extended ID (29-bit)
        TxHeaderCan.ExtId = extId;          // Zet de opgegeven ID
        TxHeaderCan.RTR = CAN_RTR_DATA;     // Data frame (geen remote request)
        TxHeaderCan.DLC = length;           // Data Length Code (payload lengte)

        // Voeg het bericht toe aan een vrije TX-mailbox en start verzending
        HAL_CAN_AddTxMessage(&hcan1, &TxHeaderCan, payload, &TxMailbox);

        // Optioneel: log de verzonden data via UART voor debug
        if(debug == 1)
        {
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
            "Send: ID=0x%08lX DLC=%u, Payload=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
			TxHeaderCan.ExtId,TxHeaderCan.DLC,
            payload[0], payload[1], payload[2], payload[3],
            payload[4], payload[5], payload[6], payload[7]
        );
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
        }
    }
    else {
        // Fout: payload te lang — voer foutafhandeling uit
        Error_Handler();
    }
}

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

uint16_t TMP117_ReadTemperatureC(uint8_t addr, uint8_t reg,uint8_t debug) {
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

// Alarm register schrijven met gewenste temperatuur.


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


/*
 * Stel het temperatuur-alarm (T<sub>nA</sub>-point) in op de TMP117
 * @param addr  7-bit I²C-adres van de TMP117 (linksshifted <<1 voor HAL)
 * @param reg   Registerpointer voor het hoge of lage alarm­punt (TMP117_REG_T_HIGH_LIMIT of TMP117_REG_T_LOW_LIMIT)
 * @param temp  Gewenste alarmtemperatuur in °C (float)
 *
 * Deze functie schrijft de temperatuur in °C om naar de ruwe 16-bit waarde
 * en gebruikt TMP117_WriteTemperatureC om het register te vullen. Er wordt
 * een debugbericht via UART verzonden om succes of falen te melden.
*/
void TMP_SetAlarmTemp(uint8_t addr, uint8_t reg, float temp)
{
    // Probeer het alarmregister te schrijven met de opgegeven temperatuur
    if (TMP117_WriteTemperatureC(addr, reg, temp) == HAL_OK) {
        // Bij succes: informeer via UART
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "TMP117: Adres 0x%02X, alarmpoint ingesteld op %.2f °C\r\n",
                           (addr >> 1), temp);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    } else {
        // Bij mislukking: log een communicatiefout
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "TMP117: Adres 0x%02X, geen I2C-communicatie bij alarmset\r\n",
                           (addr >> 1));
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }
}



/**
 * Past een of meerdere bits aan in een 16-bits register van de TMP117.
 *
 * Deze functie voert een read-modify-write uit:
 * 1. Leest het opgegeven 16-bits register.
 * 2. Wijzigt alleen de bits die overeenkomen met het meegegeven mask.
 * 3. Schrijft het gewijzigde register terug naar de TMP117.
 *
 * @param addr     7-bits I²C-adres van de TMP117 (left-shifted <<1 voor HAL-functies).
 * @param reg      Registeradres dat aangepast moet worden (bijv. TMP117_REG_CONFIG).
 * @param mask     Bitmasker van de bits die gewijzigd mogen worden (bijv. 0x00F0).
 * @param value    Gewenste waarde voor die bits (bijv. 0x0010 zet bit 4 aan).
 * @return         HAL-status (HAL_OK bij succes, anders een foutcode).
 */
HAL_StatusTypeDef TMP117_WriteMaskedRegister(uint8_t addr, uint8_t reg, uint16_t mask, uint16_t value)
{
    HAL_StatusTypeDef ret;
    uint16_t reg_val;
    uint8_t buf[2];

    // 1) Lees huidige registerwaarde
    ret = HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT,
                           (uint8_t*)&reg_val, sizeof(reg_val), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    // 2) Mask bits en zet nieuwe waarde
    reg_val = (reg_val & ~mask) | (value & mask);

    // 3) Zet MSB/LSB in buffer
    buf[0] = (uint8_t)(reg_val >> 8);
    buf[1] = (uint8_t)(reg_val & 0xFF);

    // 4) Schrijf terug naar het register
    ret = HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, sizeof(buf), HAL_MAX_DELAY);
    return ret;
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
    char buf[64];
   	int len = snprintf(buf, sizeof(buf),
                       "TMP117: Adres 0x%02X, Read Reg 0b%s (0x%04X)\r\n",
                       (addr >> 1),       // shift terug naar 7-bit adres
                       bufbin,            // binaire weergave
                       (unsigned int)ConfigRegister); // hex weergave
    	HAL_UART_Transmit(&huart2,
                      (uint8_t*)buf,
                      len,
                      HAL_MAX_DELAY);
    }
}


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


/*----------------------------------------------------------------------------*/
/*— I2C INA238 sensor communicatie functies
/*----------------------------------------------------------------------------*/


/**
 * Lees een 16-bit register uit de INA238 via I²C
 * @param addr  7-bit I²C-adres van de INA238 (linksshifted <<1 voor HAL)
 * @param reg   8-bit registerpointer binnen de INA238
 * @return      16-bit raw registerwaarde (MSB first)
 *
 * Deze functie gebruikt HAL_I2C_Mem_Read om twee bytes te lezen. Bij een
 * communicatiefout wordt een debugmelding verzonden via UART. De leesbuffer
 * wordt vervolgens samengevoegd tot één 16-bit waarde.
 */
uint16_t INA238_ReadRegister(uint8_t addr, uint8_t reg) {
    uint8_t raw[2] = {0, 0};

    // Lees 2 bytes van het opgegeven register
    if (HAL_I2C_Mem_Read(&hi2c3,
                         addr,                  // slave-adres inclusief R/W-bit
                         reg,                   // registerpointer
                         I2C_MEMADD_SIZE_8BIT,  // registeradresgrootte
                         raw,                   // ontvangbuffer
                         2,                     // aantal bytes
                         HAL_MAX_DELAY) != HAL_OK)
    {
        // Bij fout: log melding via UART
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "INA238: Adres 0x%02X: geen I2C-communicatie\r\n",
                           addr);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

    // Combineer MSB en LSB tot één 16-bit waarde
    return (uint16_t)((raw[0] << 8) | raw[1]);
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

// Amp nog doen!!


uint16_t INA238_ReadTemp(uint8_t addr, uint8_t debug){
	// 1. Lees de ruwe 16-bit waarde
	uint16_t raw = INA238_ReadRegister(addr, INA238_REG_DIETEMP);

	// 2. Shift 4 bits naar rechts (4 LSB's zijn altijd nul)
	raw >>= 4;

	// 3. Converteer naar signed 13-bit waarde
	// (bit 12 = sign bit, dus cast naar int16_t met masking)
	if (raw & 0x1000) { // Als bit 12 (sign bit) gezet is
		raw |= 0xE000;  // Zet de bovenste bits voor negatieve waarde
	}
	int16_t signed_val = (int16_t)raw;

	// 4. Bereken temperatuur (1 LSB = 0.125 °C)
	float temp = signed_val * 0.125f;

	// 5. Debug via UART
	if(debug == 1){
		char buf[64];
		int len = snprintf(buf, sizeof(buf),
						   "INA238: Adres 0x%02X, temperatuur = %.2f °C\r\n",
						   (addr >> 1), temp);
		HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
	}

	// 6. Retourneer raw (originele waarde)
	return raw;
}

HAL_StatusTypeDef INA238_WriteVoltageRegister(uint8_t addr, uint8_t reg, float voltage)
{
    // 1. Converteer spanning naar ruwe waarde (volgens 3.125 mV/bit resolutie)
    uint16_t raw = (uint16_t)lroundf(voltage / 0.003125f);

    // 2. Zet om naar big-endian payload
    uint8_t payload[2];
    payload[0] = (uint8_t)(raw >> 8);   // MSB
    payload[1] = (uint8_t)(raw & 0xFF); // LSB

    // 3. Schrijf naar opgegeven register
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        &hi2c3,               // I2C-handle
        addr,                 // Slaveadres (8-bit)
        reg,                  // Doelregister
        I2C_MEMADD_SIZE_8BIT, // Registergrootte
        payload,              // Gegevens
        sizeof(payload),      // Lengte
        HAL_MAX_DELAY
    );

    // 4. Debug via UART
    char buf[64];
    if (status == HAL_OK) {
        int len = snprintf(buf, sizeof(buf),
            "INA238: 0x%02X schrijf naar reg 0x%02X = %.3f V\r\n",
            (addr >> 1), reg, voltage);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    } else {
        int len = snprintf(buf, sizeof(buf),
            "INA238: fout bij schrijf naar reg 0x%02X (addr 0x%02X)\r\n",
            reg, (addr >> 1));
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    }

    return status;
}




/*******************************/
/*interupt handler  */
/***************************** */

uint8_t EnqueueCAN(const CAN_RxHeaderTypeDef *hdr, const uint8_t *data) {
    uint8_t next = (canHead + 1) % CAN_QUEUE_SIZE;
    if (next == canTail) {
        // buffer vol, laat ISR snel terugkeren
        return 0;
    }
    canQueue[canHead].header = *hdr;
    memcpy(canQueue[canHead].data, data, 8);
    canHead = next;
    return 1;
}

// Haal een bericht uit de ringbuffer; retourneert 1 als gelukt, 0 als leeg
uint8_t DequeueCAN(CAN_Msg_t *msg) {
    if (canTail == canHead) {
        // buffer leeg
        return 0;
    }
    *msg = canQueue[canTail];
    canTail = (canTail + 1) % CAN_QUEUE_SIZE;
    return 1;
}

void ProcessCANMessages(void) {
    CAN_Msg_t msg;
    while (DequeueCAN(&msg)) {
        uint8_t type = GET_TYPE(msg.header.ExtId);
        char buf[128];
        switch (type) {
            case 1: {  // Spanning
                VoltageMsg_t *m = (VoltageMsg_t*)msg.data;
                float voltage_v = m->value * 3.125e-3f;
                int len = snprintf(buf, sizeof(buf),
                    "Ontvangen: Spanning (ID=0x%08lX, idx=%u): %.2f V.\r\n",
                    (unsigned long)msg.header.ExtId,
                    GET_INDEX(msg.header.ExtId),
                    voltage_v);
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
                break;
            }
            case 4: {  // Temperatuur
                TempMsg_t *m = (TempMsg_t*)msg.data;
                int16_t sv = (int16_t)m->value;
                float temp_c = sv * 0.0078125f;
                int len = snprintf(buf, sizeof(buf),
                    "Ontvangen: Temperatuur (ID=0x%08lX, idx=%u): %.2f °C.\r\n",
                    (unsigned long)msg.header.ExtId,
                    GET_INDEX(msg.header.ExtId),
                    temp_c);
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
                break;
            }
            case 6: {  // Runtime
                RuntimeMsg_t *m = (RuntimeMsg_t*)msg.data;
                uint32_t s = m->runtime;
                uint32_t h = s / 3600, rem = s % 3600;
                uint32_t min = rem / 60;
                int len = snprintf(buf, sizeof(buf),
                    "Ontvangen: Uptime (ID=0x%08lX, idx=%u): %lu uur %02lu min.\r\n",
                    (unsigned long)msg.header.ExtId,
                    GET_INDEX(msg.header.ExtId),
                    h, min);
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
                break;
            }
            case 9: {  // Identificatie
                IdentMsg_t *m = (IdentMsg_t*)msg.data;
                int len = snprintf(buf, sizeof(buf),
                    "Ontvangen: Ident (ID=0x%08lX, idx=%u): verie=%u, serial=%u, board=%u.\r\n",
                    (unsigned long)msg.header.ExtId,
                    GET_INDEX(msg.header.ExtId),
                    m->version_id, m->serial_id, m->build_date);
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
                break;
            }
            default:
                // onherkend type
                break;
        }
    }
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
/*----------------------------------------------------------------------------*/
/*— CAN-bus start en activatie van RX notificatie  —*/
/*----------------------------------------------------------------------------*/
  HAL_CAN_Start(&hcan1);                   // Start van de CAN-Bus HAL

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);   // Start van Notificatie bij een ontvangen signaal.
/*----------------------------------------------------------------------------*/

/****************************************************************************************************************************************/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

/*----------------------------------------------------------------------------*/
/*— Start up van de AIM							 							 —*/
/*----------------------------------------------------------------------------*/

  AIM_INIT();
/*
  //set alarm registers


  uint16_t EEPROM_unlock = 0x8000;

  //Schrijven van EEPROM
  if (TMP117_WriteRegister(TMP117_I2C_ADDR, TMP117_REG_EEPROM_UL,EEPROM_unlock) != HAL_OK)
  {
    Error_Handler();
  }
  uint16_t ConfigRegister = TMP117_ReadRegister(TMP117_REG_EEPROM_UL);
  HAL_Delay(20);
  if (TMP117_WriteTemperatureC(TMP117_REG_T_HIGH_LIMIT,AlarmLimitHighSetPoint) != HAL_OK)
    {
      Error_Handler();
    }
  uint16_t AlarmLimitHigh = TMP117_ReadTemperatureC(TMP117_REG_T_HIGH_LIMIT);
  HAL_Delay(20);


*/


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

 /*----------------------------------------------------------------------------*/
 /*— Interupts Handeling  —*/
 /*----------------------------------------------------------------------------*/
//Handeling Temp alarm via interrupt PA8

	  ProcessCANMessages();
	  if (TmpAlertFlag == 1) { 				// lees temperatuur en stuur bericht

		  uint16_t rawtemp = TMP117_ReadTemperatureC(TMP117_I2C_ADDR, TMP117_REG_TEMP_RESULT,0);
		  int16_t signed_val = (int16_t)rawtemp;
		  float temp = signed_val * 0.0078125f;
		  char buf[80];
		  int len = snprintf(buf, sizeof(buf),
				  "Alarm! T=%.2f °C  limieten: low=%.2f, high=%.2f\r\n",
				  temp, AlarmLimitLowSetPoint, AlarmLimitHighSetPoint);
	     	HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
	     	TmpAlertFlag = 0;
	  	  }

	  if(InaAlertFlag == 1){               // Lees
	  }

	 HAL_Delay(500);




/********************************************************************/
/*- Verzenden van data via CAN-Bus									*/
 /*******************************************************************/

	  CanSendIdent(BOARD, 1, VERSION, SERIAL, BUILDDATE);
	  CanSendTemperature(BOARD, 1, TMP117_ReadTemperatureC(TMP117_I2C_ADDR, TMP117_REG_TEMP_RESULT,0));
	  CanSendUptime(BOARD, 1, DS1682_ReadElapsedTime(DS1682_I2C_ADDR), DS1682_ReadEventCounter(DS1682_I2C_ADDR));
	  CanSendVoltage(BOARD, 1,	INA238_ReadVoltage(INA238_I2C_ADDR,INA238_REG_VBUS,0),DS1682_ReadElapsedTime(DS1682_I2C_ADDR));

/********************************************************************/
/* UART testing	  													*/
/*******************************************************************/
	  DS1682_SecondsToHM_Display(DS1682_I2C_ADDR,0);
	  uint16_t VBUS1 = INA238_ReadVoltage(INA238_I2C_ADDR,INA238_REG_VBUS,1);
	  uint16_t INATEMP = INA238_ReadTemp(INA238_I2C_ADDR, 1);
	  TMP117_ReadTemperatureC(TMP117_I2C_ADDR, TMP117_REG_TEMP_RESULT,1);


  }

  /*******************************************************************************************************************************/

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 250;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */


 CAN_FilterTypeDef canfilterconfig;

 canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
 canfilterconfig.FilterBank = 10;  // anything between 0 to SlaveStartFilterBank
 canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
 canfilterconfig.FilterIdHigh = 0;
 canfilterconfig.FilterIdLow = 0x0000;
 canfilterconfig.FilterMaskIdHigh = 0;
 canfilterconfig.FilterMaskIdLow = 0x0000;
 canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
 canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
 canfilterconfig.SlaveStartFilterBank = 00;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1

 HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);


  HAL_StatusTypeDef status;
   status = HAL_CAN_Start(&hcan1);
   if (status != HAL_OK) {
        uint32_t err = HAL_CAN_GetError(&hcan1);
        char buf[80];
        int len = snprintf(buf, sizeof(buf),
            "Fout: HAL_CAN_Start() mislukt, HAL-status = %lu, CAN_Error = 0x%08lX\r\n",
            (unsigned long)status,
            (unsigned long)err);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
        Error_Handler();
    }

   status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
   if (status != HAL_OK) {
       uint32_t err = HAL_CAN_GetError(&hcan1);
       char buf[80];
       int len = snprintf(buf, sizeof(buf),
           "Fout: HAL_CAN_ActivateNotification() mislukt, HAL-status = %lu, CAN_Error = 0x%08lX\r\n",
           (unsigned long)status,
           (unsigned long)err);
       HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
       Error_Handler();
   }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10D19CE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VCALERT_Pin LinkStatus2_Pin */
  GPIO_InitStruct.Pin = VCALERT_Pin|LinkStatus2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LinkStatus0_Pin */
  GPIO_InitStruct.Pin = LinkStatus0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LinkStatus0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LinkStatus1_Pin */
  GPIO_InitStruct.Pin = LinkStatus1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LinkStatus1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TMPALERT_Pin */
  GPIO_InitStruct.Pin = TMPALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TMPALERT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		 HAL_Delay(75);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
