/*
 * linkdetect.h
 *
 *  Created on: Jun 27, 2025
 *      Author: basge
 */

#include "stm32l4xx_hal.h"

#ifndef INC_LINKDETECT_H_
#define INC_LINKDETECT_H_


typedef enum {
	LINK_STATUS0,
	LINK_STATUS1,
    LINK_STATUS2,
} Link_t;

extern uint32_t Link0HighTimestamp;
extern uint8_t  Link0ErrorFlag;
extern uint32_t Last0LinkPulseTime;

extern uint32_t Link1HighTimestamp;
extern uint8_t  Link1ErrorFlag;
extern uint32_t Last1LinkPulseTime;

extern uint32_t Link2HighTimestamp;
extern uint8_t  Link2ErrorFlag;
extern uint32_t LastLink2PulseTime;

void Link_CheckInitialLinkStatus(void);
uint8_t CheckLinkTimeout(uint32_t time_link0, uint32_t time_link1, uint32_t time_link2);
void Link_Error(Link_t err);
void Link_Recovered(Link_t link);

#endif /* INC_LINKDETECT_H_ */
