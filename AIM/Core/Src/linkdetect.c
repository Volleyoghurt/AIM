/*
 * linkdetect.c
 *
 *  Created on: Jun 28, 2025
 *      Author: basge
 */
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "linkdetect.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart2;

uint32_t Link0HighTimestamp = 0;
uint8_t  Link0ErrorFlag     = 0;
uint32_t LastLink0PulseTime = 0;

uint32_t Link1HighTimestamp = 0;
uint8_t  Link1ErrorFlag     = 0;
uint32_t LastLink1PulseTime = 0;

uint32_t Link2HighTimestamp = 0;
uint8_t  Link2ErrorFlag     = 0;
uint32_t LastLink2PulseTime = 0;

/************************************************************/
/* Link detectie functies */
/***********************************************************/

void Link_CheckInitialLinkStatus(void) {
	if (HAL_GPIO_ReadPin(LinkStatus0_GPIO_Port, LinkStatus0_Pin) == GPIO_PIN_RESET){
		Link0HighTimestamp = HAL_GetTick();
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	}

	if (HAL_GPIO_ReadPin(LinkStatus1_GPIO_Port, LinkStatus1_Pin) == GPIO_PIN_RESET){
		Link1HighTimestamp = HAL_GetTick();
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	}

	if (HAL_GPIO_ReadPin(LinkStatus2_GPIO_Port, LinkStatus2_Pin) == GPIO_PIN_RESET){
		Link2HighTimestamp = HAL_GetTick();
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	}
}

/**
 * @brief  Wordt éénmalig aangeroepen bij detectie van een langdurige hoge puls.
 */
void Link_Error(Link_t err){
		char buf[64];
	    const char *msg;
	    switch (err) {
	        case LINK_STATUS0:
	            msg = "ERROR: LinkStatus0 low >500ms\r\n";
	            // Je zou hier ook verschillende LEDs kunnen kiezen
	            // of andere GPIOs via err apart mappen
	            break;
	        case LINK_STATUS1:
	            msg = "ERROR: LinkStatus1 low >500ms\r\n";
	            break;
	        case LINK_STATUS2:
	            msg = "ERROR: LinkStatus2 low >500ms\r\n";
	            break;
	        default:
	            msg = "ERROR: Unknown link error\r\n";
	            break;
	    }
	    int len = snprintf(buf, sizeof(buf), "%s", msg);
	    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

/**
 * @brief  Wordt éénmalig aangeroepen zodra de link weer normaal functioneert.
 */
void Link_Recovered(Link_t link){
	char buf[64];
    const char *msg;
    switch (link){
 		case LINK_STATUS0:
 			msg = "INFO: LinkStatus0 activity resumed\r\n";
 			break;
 		case LINK_STATUS1:
 		    msg = "INFO: LinkStatus1 activity resumed\r\n";
 		    break;
 		case LINK_STATUS2:
 		    msg = "INFO: LinkStatus2 activity resumed\r\n";
 		    break;
 		default:
 			break;
 	}
    int len = snprintf(buf, sizeof(buf), "%s", msg);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

uint8_t CheckLinkTimeout(uint32_t time_link0, uint32_t time_link1, uint32_t time_link2){
	uint8_t error = 0;
	if (HAL_GPIO_ReadPin(LinkStatus0_GPIO_Port, LinkStatus0_Pin) == GPIO_PIN_SET){
		uint32_t elapsed_link0 = HAL_GetTick() - Link0HighTimestamp;
		if (elapsed_link0 >= time_link0 && !Link0ErrorFlag){
			Link0ErrorFlag = 1;
 			error = 1;
		}
	}
 	if (HAL_GPIO_ReadPin(LinkStatus1_GPIO_Port, LinkStatus1_Pin) == GPIO_PIN_SET){
 		uint32_t elapsed_link1 = HAL_GetTick() - Link0HighTimestamp;
 		if (elapsed_link1 >= time_link1 && !Link1ErrorFlag){
 			  Link1ErrorFlag = 1;
 			  error = 1;
 		}
 	}
 	if (HAL_GPIO_ReadPin(LinkStatus2_GPIO_Port, LinkStatus2_Pin) == GPIO_PIN_SET){
 		uint32_t elapsed_link2 = HAL_GetTick() - Link0HighTimestamp;
 		if (elapsed_link2 >= time_link2 && !Link2ErrorFlag){
 			 Link2ErrorFlag = 1;
 			 error = 1;
 		}
 	}
 	return error;
}
