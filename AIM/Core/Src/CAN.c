/*
 * CAN.c
 *
 *  Created on: Jun 28, 2025
 *      Author: basge
 */

#include "CAN.h"
#include <stdio.h>
#include "stm32l4xx_hal.h"

extern UART_HandleTypeDef huart2;

CAN_TxHeaderTypeDef TxHeaderCan;
CAN_RxHeaderTypeDef RxHeaderCan;

uint32_t TxDataCan[8];
uint8_t RxDataCan[8];

CAN_Msg_t canQueue[CAN_QUEUE_SIZE];
uint8_t canHead = 0;
uint8_t canTail = 0;

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
	CanSendMessage(id, (uint8_t*)&msg, sizeof(msg),0);
}

void CanLink(uint8_t board, uint8_t index, uint32_t status, uint32_t type, uint32_t time){
	LinkMsg_t msg;
	msg.status = status;
	msg.type = type;
	msg.timestamp = time;

	uint32_t id = CAN_EXT_ID(
		PRIO_LINK,
		board,
		TYPE_LINK,
		index
	);
	CanSendMessage(id, (uint8_t*)&msg, sizeof(msg),0);
}

typedef struct __attribute__((packed)){// __attribute__((packed)) zorgt ervoor dat er geen bitpadding ontstaat
	uint8_t message_id;					//byte 0 MCU_id
	uint8_t value;					//byte 1: Message value
	uint8_t length;					//byte 2: Message lenght
} McuMsg_t;

void CanMcuMsg(uint8_t board, uint8_t index, uint8_t id, uint8_t value, uint8_t length){
	McuMsg_t msg;
	msg.message_id = id;
	msg.value = value;
	msg.length = length;

	uint32_t id = CAN_EXT_ID(
			PRIO_MCUMSG,
			board,
			TYPE_MCUMSG,
			index);
	CanSendMessage(id, (uint8_t*)&msg, sizeof(msg),0);
}


// Can-bus zend bericht.
/**
 * Verstuur een CAN-bericht via één van de TX-mailboxen van bxCAN
 * @param extId   29-bit Extended Identifier
 * @param payload Pointer naar maximaal 8 bytes data
 * @param length  Aantal bytes in payload (0–8)
 * @param debug  Debug weergave 0 = uit, 1= aan
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
            "Send: ID=0x%08lX DLC=%lu, Payload=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
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
                    "Ontvangen: Spanning (ID=0x%08lX, idx=%lu): %.2f V.\r\n",
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
                    "Ontvangen: Temperatuur (ID=0x%08lX, idx=%lu): %.2f °C.\r\n",
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
                    "Ontvangen: Uptime (ID=0x%08lX, idx=%lu): %lu uur %02lu min.\r\n",
                    (unsigned long)msg.header.ExtId,
                    GET_INDEX(msg.header.ExtId),
                    h, min);
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
                break;
            }
            case 9: {  // Identificatie
                IdentMsg_t *m = (IdentMsg_t*)msg.data;
                int len = snprintf(buf, sizeof(buf),
                    "Ontvangen: Ident (ID=0x%08lX, idx=%lu): verie=%u, serial=%u, board=%u.\r\n",
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







