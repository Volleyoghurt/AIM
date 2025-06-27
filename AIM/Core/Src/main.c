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
#define LINK_HIGH_TIMEOUT_MS 500

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

UART_HandleTypeDef huart1;
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
static void MX_USART1_UART_Init(void);
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
uint32_t linkHighTimestamp = 0;
uint8_t  linkErrorFlag     = 0;
uint32_t lastLinkPulseTime = 0;



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

    // 1. Haal de CAN-header en de bijbehorende data (max 8 bytes) op uit FIFO0
    //    Dit wordt aangeroepen zodra een bericht binnenkomt in FIFO0 (interrupt-driven)
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msg.header, msg.data);

    // 2. Plaats het bericht in de software-queue (bijv. ringbuffer of wachtrij)
    //    EnqueueCAN retourneert meestal niets, dus als de queue vol is wordt het bericht genegeerd
    EnqueueCAN(&msg.header, msg.data);
}

/**
 * @brief GPIO External Interrupt Callback.
 *
 * Deze functie handelt EXTI-interrupts af van meerdere GPIO-pinnen:
 * - LinkStatus1_Pin: Detecteert verbindingsproblemen via een puls.
 * - TMPALERT_Pin: Waarschuwt wanneer TMP117 temperatuur buiten limieten meet.
 * - VCALERT_Pin: Signaleert spannings- of stroomfouten via INA238.
 *
 * Bij een langdurige hoog-signaal op LinkStatus1 (> LINK_HIGH_TIMEOUT_MS) wordt een foutmelding gegenereerd.
 * Als de activiteit herstelt (puls korter dan timeout), wordt de fout automatisch gecleared.
 *
 * @param GPIO_Pin De GPIO-pin waarvoor de interrupt plaatsvond.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == LinkStatus1_Pin)
	    {
	        GPIO_PinState state = HAL_GPIO_ReadPin(LinkStatus1_GPIO_Port, LinkStatus1_Pin);

	        if (state == GPIO_PIN_SET)
	        {
	            // Start van hoog-signaal detecteren
	            linkHighTimestamp = HAL_GetTick();
	        }
	        else
	        {
	            // Puls is weer laag → reset fout als het een korte puls was
	            if (linkErrorFlag)
	            {
	                linkErrorFlag = 0;
	                OnLinkRecovered();
	            }
	        }
        // LED-blink bij ontvangen puls
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
    else if (GPIO_Pin == TMPALERT_Pin)
    {
        TmpAlertFlag = 1;
    }
    else if (GPIO_Pin == VCALERT_Pin)
    {
        InaAlertFlag = 1;
    }
    else
    {
        __NOP();
    }
}

/**
 * @brief Converteert een spanningswaarde naar een 16-bit registerwaarde op basis van opgegeven LSB.
 *
 * Deze functie rekent een spanning (of stroom) in volt om naar een digitale waarde
 * zoals verwacht door registers van de INA238 (of vergelijkbare IC's).
 * De LSB bepaalt de resolutie: hoeveel volt één bit voorstelt.
 *
 * @param voltage   De gewenste waarde in volt (bijv. 16.0 voor 16V of 0.5 voor 0.5A).
 * @param lsb       De resolutie in volt per bit (bijv. 0.003125 voor spanning, 0.0005 voor stroom).
 *
 * @return uint16_t De afgeronde digitale waarde die past bij het registerformaat van 16 bits.
 */
uint16_t ConvertVoltageToRaw(float voltage, float lsb)
{
    return (uint16_t)lroundf(voltage / lsb);
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

void CanSendCurrent(uint8_t board, uint8_t index, uint32_t current, uint32_t time){
	CurrentMsg_t msg;
	msg.value = current;
	msg.status = 0x00;
	msg.timestamp = time;

	uint32_t id = CAN_EXT_ID(
		PRIO_STROOM,
		board,
		TYPE_STROOM,
		index
	);
	CanSendmessage(id, (uint8_t*)&msg, sizeof(msg),0);
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
 * @param  temp_crit     Kritieke temperatuurgrens (°C)
 */
void TMP117_Init(uint8_t addr,
                 uint16_t config,
                 float temp_high,
                 float temp_low)
{
    // TMP117 gebruikt 0.0078125°C/LSB → 1/128
    const float lsb = 0.0078125f;

    // Zet °C om naar raw registerwaarden (2-complement formaat)
    uint16_t raw_high  = (int16_t)(temp_high  / lsb);
    uint16_t raw_low   = (int16_t)(temp_low   / lsb);

    // Struct voor reg/waarde-koppels
    typedef struct {
        uint8_t reg;
        uint16_t value;
    } TMP117_WritePair_t;

    // Lijst met initialisatiewaarden
    const TMP117_WritePair_t init_config[] = {
        { TMP117_REG_CONFIG, 		config     },
        { TMP117_REG_T_HIGH_LIMIT,  raw_high   },
        { TMP117_REG_T_LOW_LIMIT,   raw_low    },
    };

    // Registreer alle waarden in volgorde
    for (uint8_t i = 0; i < sizeof(init_config) / sizeof(init_config[0]); i++) {
        TMP117_WriteRegister(addr,
                             init_config[i].reg,
                             init_config[i].value);
    }
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
                 uint16_t config,
                 uint16_t adc_config,
                 uint16_t shunt_cal,
                 uint16_t diag_alert,
                 float solv_threshold,
                 float sulv_threshold,
                 float bovl_threshold,
                 float buvl_threshold)
{
    float lsb_voltage = 0.003125f;  // Resolutie van de spanningsdrempelregisters (datasheetwaarde)

    // Zet de spanningsdrempels om naar 16-bit registerwaarden
    uint16_t solv_raw = ConvertVoltageToRaw(solv_threshold, lsb_voltage);
    uint16_t sulv_raw = ConvertVoltageToRaw(sulv_threshold, lsb_voltage);
    uint16_t bovl_raw = ConvertVoltageToRaw(bovl_threshold, lsb_voltage);
    uint16_t buvl_raw = ConvertVoltageToRaw(buvl_threshold, lsb_voltage);

    // Datastructuur om registeradres + gewenste waarde te groeperen
    typedef struct {
        uint8_t reg;     // Registeradres in de INA238
        uint16_t value;  // Te schrijven 16-bit waarde
    } INA238_WritePair_t;

    // Initialisatiearray met alle te schrijven instellingen
    const INA238_WritePair_t init_config[] = {
        { INA238_REG_CONFIG,     config     },
        { INA238_REG_ADC_CONFIG, adc_config },
        { INA238_REG_VSHUNT_CAL, shunt_cal  },
        { INA238_REG_DIAG_ALERT, diag_alert },
        { INA238_REG_SOLV,       solv_raw   },
        { INA238_REG_SULV,       sulv_raw   },
        { INA238_REG_BOLV,       bovl_raw   },
        { INA238_REG_BULV,       buvl_raw   },
    };

    // Doorloop alle init-instellingen en schrijf elke waarde naar het bijbehorende register
    for (uint8_t i = 0; i < sizeof(init_config) / sizeof(init_config[0]); i++) {
        INA238_WriteRegister(addr,                // Doelapparaat (slave-adres)
                             init_config[i].reg,  // Registeradres
                             init_config[i].value // Te schrijven waarde
        );
    }
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

/************************************************************/
/* Link detectie functies */
/***********************************************************/

void CheckInitialLinkStatus(void) {
    HAL_Delay(10);
	if (HAL_GPIO_ReadPin(LinkStatus1_GPIO_Port, LinkStatus1_Pin) == GPIO_PIN_RESET){
        linkHighTimestamp = HAL_GetTick();
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    }
}


/**
 * @brief  Wordt éénmalig aangeroepen bij detectie van een langdurige hoge puls.
 */
void OnLinkError(void)
{
    // Bijvoorbeeld: stuur foutmelding via UART en zet error-LED
	char buf[64];
	int len = snprintf(buf, sizeof(buf),
							"ERROR: LinkStatus1 high >500ms\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);

    // Schakel een fout-LED in (stel LD2 is error-led)
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Wordt éénmalig aangeroepen zodra de link weer normaal functioneert.
 */
void OnLinkRecovered(void)
{
 	char buf[64];
	int len = snprintf(buf, sizeof(buf),
			"INFO: LinkStatus1 activity resumed\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    // Zet error-LED uit
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}


void CheckLinkTimeout(void)
{
    if (HAL_GPIO_ReadPin(LinkStatus1_GPIO_Port, LinkStatus1_Pin) == GPIO_PIN_SET)
    {
        uint32_t elapsed = HAL_GetTick() - linkHighTimestamp;
        if (elapsed >= 500 && !linkErrorFlag)
        {
            linkErrorFlag = 1;
            OnLinkError();  // Foutmelding
        }
    }
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


// init van de AIM bij eerste opstart MCU

void AIM_INIT(void)
{
	//Start bericht
	UART_MSG();
	TMP117_Init(TMP117_I2C_ADDR, 0x0D80, 27, 24);
	INA238_Init(INA238_I2C_ADDR, 0x0000, 0xFB68, 0x4096, 0x0001, 0.16384, 0, 0, 0.16384);


	uint16_t AlarmLimitLow = TMP117_ReadTemperatureC(TMP117_I2C_ADDR, TMP117_REG_T_LOW_LIMIT,0);
	uint16_t AlarmLimitHigh = TMP117_ReadTemperatureC(TMP117_I2C_ADDR, TMP117_REG_T_HIGH_LIMIT,0);






		TMP117_Display_Register (TMP117_I2C_ADDR, TMP117_REG_CONFIG);
		//}


	 	 // Send identificatie van het board
	 	 CanSendIdent(BOARD, 1, VERSION, SERIAL, BUILDDATE);

	 	CheckInitialLinkStatus();

}


void UART_MSG(void)
{
	char buf[80];
	int len = snprintf(buf, sizeof(buf),"*************AIM gestart!**********\n\r");
  	HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
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
  MX_USART1_UART_Init();
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

	  CheckLinkTimeout();  // ← controleer of het signaal al te lang hoog blijft

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

	  if(InaAlertFlag == 1){
		/*

		  uint16_t rawvoltage = INA238_ReadVoltage(INA238_I2C_ADDR,INA238_REG_VBUS,0);
		  uint16_t rawcurrent = INA238_ReadCurrent(INA238_I2C_ADDR, INA238_REG_CURRENT, 0);

		  int16_t signed_val_V = (int16_t)rawvoltage;
		  float voltage = signed_val * 3.125e-3f;  // V

		  int16_t signed_val_I = (int16_t)rawcurrent;
		  float current = signed_val * 3.125e-3f;  // V
		*/


		  // Lees
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
	  uint16_t Vbus1 = INA238_ReadVoltage(INA238_I2C_ADDR,INA238_REG_VBUS,1);
	  uint16_t Current = INA238_ReadCurrent(INA238_I2C_ADDR, INA238_REG_CURRENT, 1);
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
