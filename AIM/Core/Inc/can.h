/*
 * can.h
 *
 *  Created on: Jun 10, 2025
 *      Author: basge
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

/*----------------------------------------------------------------------------*/
/*— Macros voor 29-bit Extended CAN-ID —*/
/*----------------------------------------------------------------------------*/
#define CAN_EXT_ID(prio, board, type, idx) \
    ( ((uint32_t)(prio)  & 0x7)  << 26  | \
      ((uint32_t)(board) & 0x3F) << 20  | \
      ((uint32_t)(type)  & 0x3F) << 14  | \
      ((uint32_t)(idx)   & 0x3F) << 8 )

#define GET_PRIO(id)   (((id) >> 26) & 0x7)
#define GET_BOARD(id)  (((id) >> 19) & 0x7F)
#define GET_TYPE(id)   (((id) >> 14) & 0x1F)
#define GET_INDEX(id)  (((id) >> 10) & 0x0F)

/* Vooraf gedefinieerde prioriteiten */
#define PRIO_STATUS    	1
#define PRIO_IDENT     	2
#define PRIO_LINK      	3
#define PRIO_SPANNING  	4
#define PRIO_STROOM		5
#define PRIO_TEMP		6
#define PRIO_RUNTIME	7
#define PRIO_LUCHTV		8

/*vooraf gedefinieerde taken */
#define TYPE_SPANNING 1
#define TYPE_STROOM 2
#define TYPE_LINK 3
#define TYPE_TEMP 4
#define TYPE_LUCHTV 5
#define TYPE_RUNTIME 6
#define TYPE_STATUS 8
#define TYPE_IDENT 9

/*interupt handeling */

#define CAN_QUEUE_SIZE 16

// zorg dat je telt vanuit 0 ipv 1
/*----------------------------------------------------------------------------*/
/*— CAN-bus Berichtenstructuren (8 bytes payload) —*/
/*----------------------------------------------------------------------------*/

typedef struct __attribute__((packed)) {     // __attribute__((packed)) zorgt ervoor dat er geen bitpadding ontstaat
    uint16_t status_flags;   // Byte 0–1: Bitveld voor subsystem status
    uint8_t  error_code;     // Byte 2: Foutcode volgens fouttabel
    uint8_t  warning_code;   // Byte 3: Waarschuwingscode (bijv. temp/spanning)
    uint32_t timestamp;   	 // Byte 4–7: Tijdstempel
} StatusMsg_t;

typedef struct __attribute__((packed)) { // __attribute__((packed)) zorgt ervoor dat er geen bitpadding ontstaat
	uint8_t  version_id;   		//byte 0: Firmwareversie of configuratieversie
	uint8_t  serial_id; 		//byte 1-3: Unieke identifier of deel van serienummer
	uint8_t  build_date;  		//byte 4-6 Bouw datum
} IdentMsg_t;

typedef struct __attribute__((packed)) { // __attribute__((packed)) zorgt ervoor dat er geen bitpadding ontstaat
	uint8_t  status;			//byte 0: Linkstatus 0= geen verbinding, 1= wel verbinding
	uint8_t  type;				//byte 1: type verbinding volgens type tabel
	uint32_t timestamp;			//byte 2-5: Tijd sinds laatste succesvolle communicatie
} LinkMsg_t;

typedef struct __attribute__((packed)) { // __attribute__((packed)) zorgt ervoor dat er geen bitpadding ontstaat
	uint16_t value;					//byte 0-1 16 bit spanning uitlezing (3.125 mV/LSB)
	uint8_t  status;				//byte 2: Sensorstatus
	uint32_t timestamp;				//byte 3-6 Meetmoment
} VoltageMsg_t;

typedef struct __attribute__((packed)) { // __attribute__((packed)) zorgt ervoor dat er geen bitpadding ontstaat
	uint16_t value;				//byte 0-1 16 bit stroom uitlezing
	uint8_t  status;				//byte 2: Sensorstatus
	uint32_t timestamp;			//byte 3-6 Meetmoment
} CurrentMsg_t;

typedef struct __attribute__((packed)) { // __attribute__((packed)) zorgt ervoor dat er geen bitpadding ontstaat
	uint16_t value;				//byte 0-1 Temperatuur (7.8125 m°C/LSB).
	uint8_t  status;				//byte 2: Sensorstatus
	uint32_t timestamp;			//byte 3-6 Meetmoment
} TempMsg_t;

typedef struct __attribute__((packed)) { // __attribute__((packed)) zorgt ervoor dat er geen bitpadding ontstaat
	uint32_t runtime;				//byte 0-4 Runtime.
	uint8_t reset_count;			//byte 5-7: aantal keer reset
} RuntimeMsg_t;



#define CAN_QUEUE_SIZE 16

typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
} CAN_Msg_t;

static CAN_Msg_t canQueue[CAN_QUEUE_SIZE];
static volatile uint8_t canHead = 0, canTail = 0;

// Prototype (zorg dat dit één keer staat, vóór gebruik)
static uint8_t EnqueueCAN(const CAN_RxHeaderTypeDef *hdr, const uint8_t *data);
static uint8_t DequeueCAN(CAN_Msg_t *msg);






void SendTemperature(uint8_t board, uint8_t index, uint16_t temperature);
void SendIdent(uint8_t board, uint8_t index, uint8_t id, uint8_t serid, uint8_t bdate);
void SendUptime(uint8_t board, uint8_t index, uint32_t uptime_seconds, uint32_t reset);
void SendVoltage(uint8_t board, uint8_t index, uint32_t voltage, uint32_t time);


#endif /* INC_CAN_H_ */
