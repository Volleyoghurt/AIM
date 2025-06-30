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

uint32_t linkHighTimestamp = 0;
uint8_t  linkErrorFlag     = 0;
uint32_t lastLinkPulseTime = 0;

/************************************************************/
/* Link detectie functies */
/***********************************************************/

void Link_ETH_CheckInitialLinkStatus(void) {
    HAL_Delay(10);
	if (HAL_GPIO_ReadPin(LinkStatus1_GPIO_Port, LinkStatus1_Pin) == GPIO_PIN_RESET){
        linkHighTimestamp = HAL_GetTick();
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    }
}

/**
 * @brief  Wordt éénmalig aangeroepen bij detectie van een langdurige hoge puls.
 */
void Link_ETH_OnLinkError(void)
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
void Link_ETH_OnLinkRecovered(void)
{
 	char buf[64];
	int len = snprintf(buf, sizeof(buf),
			"INFO: LinkStatus1 activity resumed\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    // Zet error-LED uit
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}


void Link_ETH_CheckLinkTimeout(void)
{
    if (HAL_GPIO_ReadPin(LinkStatus1_GPIO_Port, LinkStatus1_Pin) == GPIO_PIN_SET)
    {
        uint32_t elapsed = HAL_GetTick() - linkHighTimestamp;
        if (elapsed >= 500 && !linkErrorFlag)
        {
            linkErrorFlag = 1;
            Link_ETH_OnLinkError();  // Foutmelding
        }
    }
}
