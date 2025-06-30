/*
 * linkdetect.h
 *
 *  Created on: Jun 27, 2025
 *      Author: basge
 */

#include "stm32l4xx_hal.h"

#ifndef INC_LINKDETECT_H_
#define INC_LINKDETECT_H_

extern uint32_t linkHighTimestamp;
extern uint8_t  linkErrorFlag;
extern uint32_t lastLinkPulseTime;

void Link_ETH_CheckInitialLinkStatus(void);
void Link_ETH_OnLinkError(void);
void Link_ETH_OnLinkRecovered(void);
void Link_ETH_CheckLinkTimeout(void);



#endif /* INC_LINKDETECT_H_ */
