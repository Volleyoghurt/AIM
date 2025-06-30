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
#include "linkdetect.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VERSION 01
#define BUILDDATE __DATE__
#define SERIALNR 1337
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
uint8_t RX_Buffer_I2C[1]; //I2C RX variable
uint8_t rx_buff_UART[10];
char buf[64];
uint8_t TempReady 		= 0;
uint8_t TmpAlertFlag 	= 0;
uint8_t InaAlertFlag 	= 0;
static uint8_t printed = 0;
char TempBuf[64];

uint32_t pulseStart = 0;
uint32_t pulseWidth = 0;

/* Settings:--------------------------------------*/
static float AlarmLimitHighSetPoint 	= 25.0;    //nog fixen
static float AlarmLimitLowSetPoint 	= 24.0;

static State_t state = STATE_INIT;
static uint32_t lastTick = 0;
static uint16_t rawTemp;
static uint32_t elapsedTime;
static uint16_t rawCurrent;
static uint16_t rawVoltage;


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
	        GPIO_PinState state_int = HAL_GPIO_ReadPin(LinkStatus1_GPIO_Port, LinkStatus1_Pin);

	        if (state_int == GPIO_PIN_SET)
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
	                Link_ETH_OnLinkRecovered();
	            }
	        }
        // LED-blink bij ontvangen puls
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
    else if (GPIO_Pin == TMPALERT_Pin)
    {
        state = STATE_ERROR;
    	TmpAlertFlag = 1;
    }
    else if (GPIO_Pin == VCALERT_Pin)
    {
        state = STATE_ERROR;
    	InaAlertFlag = 1;
    }
    else
    {
        __NOP();
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //HAL_UART_Receive_IT(&huart1, rx_buff_UART, 10); //You need to toggle a breakpoint on this line!
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

// init van de AIM bij eerste opstart MCU

void AIM_INIT(void)
{
	//Start bericht
	UART_MSG();

	TMP117_Init(TMP117_I2C_ADDR,0b0000000000110000,25.0, 24.0);
	//HAL_Delay(50);
	//INA238_Init(INA238_I2C_ADDR, 0x0000, 0xFB68, 0x4096, 0x0001, 0.16384, 0, 0, 0.16384);
	//HAL_Delay(50);

	TMP117_Display_Register(TMP117_I2C_ADDR, TMP117_REG_CONFIG);
	TMP117_ReadTemperatureC(TMP117_I2C_ADDR, TMP117_REG_T_HIGH_LIMIT, 1);
	TMP117_ReadTemperatureC(TMP117_I2C_ADDR, TMP117_REG_T_LOW_LIMIT, 1);

	// Send identificatie van het board
	CanSendIdent(BOARD, 1, VERSION, SERIALNR, BUILDDATE);
	Link_ETH_CheckInitialLinkStatus();
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
  HAL_UART_Receive_IT(&huart1, rx_buff_UART, 10);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

/*----------------------------------------------------------------------------*/
/*— Start up van de AIM							 							 —*/
/*----------------------------------------------------------------------------*/

  state = STATE_INIT;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch (state)
	          {
	              case STATE_INIT:
	                  AIM_INIT();           // jouw init-flow
	                  state = STATE_READ;
	                  break;

	              case STATE_READ:
	                  // 1) Lees temperature en runtime:
	                  rawTemp     = TMP117_ReadRegister(TMP117_I2C_ADDR, TMP117_REG_TEMP_RESULT);
	                  TMP117_Display_Register(TMP117_I2C_ADDR,TMP117_REG_CONFIG);
	                  elapsedTime = DS1682_ReadElapsedTime(DS1682_I2C_ADDR);
	                  rawVoltage  = INA238_ReadRegister(INA238_I2C_ADDR, INA238_REG_VBUS);
	                  rawCurrent  = INA238_ReadCurrent(INA238_I2C_ADDR, INA238_REG_CURRENT, 0);
	                  state = STATE_TRANSMIT;
	                  break;

	              case STATE_TRANSMIT:
	                  // 2) Verstuur via CAN-BUS:
	                  CanSendTemperature(BOARD, 1, rawTemp);
	                  CanSendUptime   (BOARD, 1, elapsedTime, DS1682_ReadEventCounter(DS1682_I2C_ADDR));
	                  CanSendCurrent  (BOARD, 1, rawCurrent, HAL_GetTick());
	            	  CanSendVoltage(BOARD, 1,	INA238_ReadVoltage(INA238_I2C_ADDR,INA238_REG_VBUS,0),DS1682_ReadElapsedTime(DS1682_I2C_ADDR));
	            	  state = STATE_DISPLAY;
	                  break;

	              case STATE_DISPLAY:
	            	  //4) geeft waarde weer op display
	            	  DS1682_SecondsToHM_Display(DS1682_I2C_ADDR,1);
	            	  uint16_t Vbus1 = INA238_ReadVoltage(INA238_I2C_ADDR,INA238_REG_VBUS,1);
	            	  uint16_t Current = INA238_ReadCurrent(INA238_I2C_ADDR, INA238_REG_CURRENT, 1);
	            	  uint16_t INATEMP = INA238_ReadTemp(INA238_I2C_ADDR, 1);
	            	  uint16_t TMPTEMP = TMP117_ReadTemperatureC(TMP117_I2C_ADDR, TMP117_REG_TEMP_RESULT,1);

	            	  lastTick = HAL_GetTick();  // tijdstip van ingang in wachtperiode
	            	  state = STATE_WAIT;
	            	  break;

	              case STATE_WAIT:

	            	  if ((HAL_GetTick() - lastTick) >= 1000) {
	            		   state = STATE_READ;
	            	  }
	            	  break;

	              case STATE_ERROR:
	            	  if (TmpAlertFlag) {
	            		  if(TMP117_ReadRegister(TMP117_I2C_ADDR, TMP117_REG_CONFIG) & ((1 << 15) | (1 << 14))) // controleer of een van de twee alerts hoog zijn.
	            		  {
	            				 TMP117_Temp_Alert(TMP117_I2C_ADDR);
	            		  }
	            			 TmpAlertFlag = 0;
	            		  }

	            		  if(InaAlertFlag){

	            		  }

	            		  state = STATE_READ;
	            	  // 5) Handel Error binnen syteem af.
	            	  break;
	          }

		Link_ETH_CheckLinkTimeout();  // ← controleer of het signaal al te lang hoog blijft

/*----------------------------------------------------------------------------*/
 /*— Interupts Handeling  —*/
 /*----------------------------------------------------------------------------*/
//Handeling Temp alarm via interrupt PA8

	 ProcessCANMessages();

/*
		    uint16_t status = INA238_ReadRegister(INA238_I2C_ADDR, INA238_REG_DIAG_ALERT);

		    // 2) Geen flags? Niets te doen.
		    if (status == 0)
		        return;

		    // 3) Controleer per bit en bel handler / log
		    if (status & INA238_ALERT_MEM_ERROR) {
		        // geheugen‐checksum fout
		        InaMemError();   // bijvoorbeeld: LED rood zetten, UART-log, enz.
		    }
		    if (status & INA238_ALERT_MATH_OVERFLOW) {
		        // arithmetic overflow: current/power ongeldig
		        InaMathOverflow();
		    }
		    if (status & INA238_ALERT_TEMP_OVER) {
		        // temperatuurdrempel overschreden
		        InaTempOver();
		    }
		    if (status & INA238_ALERT_SHUNT_OVER) {
		        // shuntspanning te hoog
		        InaShuntOver();
		    }
		    if (status & INA238_ALERT_SHUNT_UNDER) {
		        // shuntspanning te laag
		        InaShuntUnder();
		    }
		    if (status & INA238_ALERT_BUS_OVERVOLT) {
		        // bussenspanning te hoog
		        InaBusOver();
		    }
		    if (status & INA238_ALERT_BUS_UNDERVOLT) {
		        // bussenspanning te laag
		        InaBusUnder();
		    }
		    if (status & INA238_ALERT_POWER_OVER) {
		        // vermogensdrempel overschreden
		        InaPowerOver();
		    }
		    if (status & INA238_ALERT_CONVERSION_DONE) {
		        // conversie voltooid
		        InaConversionDone();
		    }
		}
*/
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

	 HAL_Delay(500);

	 uint16_t rawvolt INA238_ReadRegister(INA238_I2C_ADDR, INA238_REG_VBUS);
	 			uint16_t rawvoltol INA238_ReadRegister(INA238_I2C_ADDR, INA238_REG_BOLV,0);

	 			int16_t signed_val = (int16_t)rawvolt;
	 			float volt = signed_val * 3.125e-3f;

	 			int16_t signed_val_OL = (int16_t)rawvoltul;
	 			float volt_OL = signed_val_OL * 3.125e-3f;

	 			char buf[80];
	 			nt len = snprintf(buf, sizeof(buf),
	 						  "Alarm! High voltage! Voltage=%.2fV  limiet: high=%.2f\r\n",
	 						  volt, volt_OL);
	 			HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
	 			InaAlertFlag = 0;
*/



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
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
