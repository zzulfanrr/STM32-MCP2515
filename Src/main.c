/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "CAN_SPI.h"
#include "MCP2515.h"

#include <stdio.h>
#include <string.h>

#include "stm32wlxx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TIMEOUT_MS 1000  // Timeout untuk komunikasi SPI (dalam ms)

static uint32_t lastStateChangeTime = 0;
static const uint32_t debounceDelay = 200;

static uint32_t lastWakeTime = 0;

// Declare your global or external variables appropriately
static uint32_t prevTX = 0;  // Variable to store the last transmit time
static uint32_t invlTX = 1000; // Example: 1 second interval
#define MAX_DATA_SETS 4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Define Car states */
typedef enum {
	CAR_OFF, CAR_ACC_ON, CAR_ENGINE_ON
} CarState;

static CarState currentState = CAR_OFF;

/* Define PID codes */
#define PID_ENGINE_LOAD 0x04
#define PID_ENGINE_COOLANT_TEMPERATURE 0x05
#define PID_SHORT_TERM_FUEL_TRIM 0x06
#define PID_MANIFOLD_ABSOLUTE_PRESSURE 0x0B
#define PID_ENGINE_SPEED 0x0C
#define PID_VEHICLE_SPEED 0x0D
#define PID_INTAKE_AIR_TEMPERATURE 0x0F
#define PID_THROTTLE_POSITION 0x11
#define PID_O2_SENSOR_2 0x15
#define PID_RUN_TIME 0x1F
#define PID_CONTROL_MODULE_VOLTAGE 0x42

#define RES_ID 0x98DA01F1
#define REQ_ID 0x98DB33F1
/* Global variables for PID */
static int manifold_absolute_pressure;
static int engine_speed;
static int vehicle_speed;
static int intake_air_temperature;
static int control_module_voltage;

/* OBD2 data sets request */
static const uint8_t dataSets[][8] = {
		{ 0x02, 0x01, 0x42, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },  // Control Module Voltage
		{ 0x02, 0x01, 0x0B, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA }, // Manifold Absolute Pressure
		{ 0x02, 0x01, 0x0C, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },  // Engine Speed
		{ 0x02, 0x01, 0x0D, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },  // Vehicle Speed
		{ 0x02, 0x01, 0x0F, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA }, // Intake Air Temperature
//		{ 0x02, 0x01, 0x42, 0x00, 0x12, 0xAA, 0xAA, 0xAA },  // Control Module Voltage
		};

static bool responseReceived[MAX_DATA_SETS] = { false };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
static uint32_t delay = 250;
SPI_HandleTypeDef hspi1;

uCAN_MSG txMessage;
uCAN_MSG rxMessage;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

void updateCarState(void);
void handleCarState(void);
void sendOBDRequests(void);
void decodePID(uint8_t pid, uint8_t a, uint8_t b, uint8_t c, uint8_t d,
		uint8_t e);
void enterSleepMode(void);
void logPID(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Initialize leds */
	BSP_LED_Init(LED_BLUE);
	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_RED);

	/* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
	BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
	BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);
	BSP_PB_Init(BUTTON_SW3, BUTTON_MODE_EXTI);

	/* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
	BspCOMInit.BaudRate = 115200;
	BspCOMInit.WordLength = COM_WORDLENGTH_8B;
	BspCOMInit.StopBits = COM_STOPBITS_1;
	BspCOMInit.Parity = COM_PARITY_NONE;
	BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
	if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
		Error_Handler();
	}

	/* USER CODE BEGIN BSP */

	/* -- Sample board code to send message over COM1 port ---- */
	printf("\n\r");
	printf("STM32 x MCP2515 : OBD2 Scanner\n\r");

	/* -- Sample board code to switch on leds ---- */
	BSP_LED_On(LED_BLUE);
	BSP_LED_On(LED_GREEN);
	BSP_LED_On(LED_RED);

	/* USER CODE END BSP */

	/* Boot CPU2 */
	HAL_PWREx_ReleaseCore(PWR_CORE_CPU2);

	/* MCP2515 & SPI Initialization */
	int ret;
	ret = CANSPI_Initialize();
	if (ret < 0) {
		printf("Initialize CAN SPI Failed... Code: (%d)\n\r", ret);
		Error_Handler();
	} else {
		printf("MCP2515 initialized successfully!\n\r");
	}

	/* Set MCP2515 to Normal mode */
	if (!MCP2515_SetNormalMode()) {
		printf("Failed to set Normal mode!\n\r");
	} else {
		printf("MCP2515 set to Normal mode.\n\r");
	}

//	/* Set MCP2515 to Loopback mode */
//	if (!MCP2515_SetLoopbackMode()) {
//		printf("Failed to set Loopback mode!\n\r");
//	} else {
//		printf("MCP2515 set to Loopback mode.\n\r");
//	}

	/* Set initial LED states to Off */
	BSP_LED_Off(LED_BLUE);  // Will use for CAN activity
	BSP_LED_Off(LED_GREEN); // Will use for successful communication
	BSP_LED_On(LED_RED);    // Will use for errors/no communication

	/* Print CSV header for logging */
	printf("#,timestamp,carstate,MAP,RPM,VSS,IAT,CMV\n\r");

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		  // Update car state
		  updateCarState();

		  // Handle current state
		  handleCarState();

//		// Transmit the message sequentially
//		for (int i = 0; i < sizeof(dataSets) / sizeof(dataSets[0]); i++) {
//			// Create CAN message for the current dataset
//			txMessage.frame.id = REQ_ID;
//			txMessage.frame.dlc = 8;
//			memcpy(&txMessage.frame.data0, dataSets[i], 8);
//
//			// Send the message
//			if (CANSPI_Transmit(&txMessage)) {
//				printf("\n\rSent OBD request for PID: 0x%02X\n\r",
//						dataSets[i][2]);
//
//				// Wait for a response
//				uint32_t startTime = HAL_GetTick();
//
//				while (HAL_GetTick() - startTime < TIMEOUT_MS) { // Timeout set to 1000 ms
//					// Check if a message is received
//					if (CANSPI_Receive(&rxMessage)) {
//
//						decodePID(rxMessage.frame.data2, rxMessage.frame.data3,
//								rxMessage.frame.data4, rxMessage.frame.data5,
//								rxMessage.frame.data6, rxMessage.frame.data7);
//
////						if (rxMessage.frame.id == REQ_ID) {
////							printf(
////									"Received Message: Data = {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}\n\r",
////									rxMessage.frame.data0,
////									rxMessage.frame.data1,
////									rxMessage.frame.data2,
////									rxMessage.frame.data3,
////									rxMessage.frame.data4,
////									rxMessage.frame.data5,
////									rxMessage.frame.data6,
////									rxMessage.frame.data7);
////							responseReceived = true;
////							break;  // Exit loop once response is received
////						}
//					}
//				}
//			} else {
//				printf("Failed to send request for PID: 0x%02X\n\r",
//						dataSets[i][2]);
//			}
//
//			// Small delay between requests
//			HAL_Delay(50);
//		}

		/* -- Sample board code for User push-button in interrupt mode ---- */
//    BSP_LED_Toggle(LED_BLUE);
//    HAL_Delay(delay);
//
//    BSP_LED_Toggle(LED_GREEN);
//    HAL_Delay(delay);
//
//    BSP_LED_Toggle(LED_RED);
//    HAL_Delay(delay);
//
	}

	/* USER CODE END WHILE */
	/* USER CODE END 3 */
}

// Implementation of car state update
void updateCarState(void) {
	static bool firstRun = true;
	uint32_t currentMillis = HAL_GetTick();
	CarState newState = currentState;

	if (firstRun) {
		sendOBDRequests();  // Call function to send OBD requests, fist PID: 0x42 (Control Module Voltage)
		firstRun = false;
	}

	// Update the new state based on control module voltage
	if (control_module_voltage > 13) {  // 13.5 - 14.7
		newState = CAR_ENGINE_ON;
	} else if (control_module_voltage > 12) {  // 12.0 - 12.6
		newState = CAR_ACC_ON;
	} else {
		newState = CAR_OFF;
	}

	// Debouncing
	if (newState != currentState
			&& (currentMillis - lastStateChangeTime > debounceDelay)) {
		currentState = newState;
		lastStateChangeTime = currentMillis;

		printf("Car state changed to: %d\n", currentState);

		// Check if the car state = off, enter sleep mode
		if (currentState == CAR_OFF) {
			enterSleepMode();
		}
	}
}

// Implementation of Handle Car State
void handleCarState(void) {
    uint32_t currentTime = HAL_GetTick();

    switch (currentState) {
    case CAR_OFF:
        if (currentTime - lastWakeTime > 10000) { // 10 seconds
            printf("CAR OFF\n\r");
            enterSleepMode();
        }
        break;

    case CAR_ACC_ON:
    case CAR_ENGINE_ON:
        if (currentTime - prevTX >= invlTX) {
            printf("CAR ON\n\r");
            sendOBDRequests();
            prevTX = currentTime;
        }
        break;
    }

    if (currentTime < lastWakeTime) {
        lastWakeTime = currentTime;
    }
}


// Implementation of OBD requests sender
void sendOBDRequests(void) {
	bool receivedResponse = false;

	for (int i = 0; i < sizeof(dataSets) / sizeof(dataSets[0]); i++) {
		// Create CAN message (ID, DLC, Data Message)
		uCAN_MSG txMessage;
		txMessage.frame.id = REQ_ID;
		txMessage.frame.dlc = 8;
		memcpy(&txMessage.frame.data0, dataSets[i], 8);

		// Send the CAN message
		if (CANSPI_Transmit(&txMessage)) {
			printf("\n\rSent OBD request for PID: 0x%02X\n\r", dataSets[i][2]);

			// Wait for a response
			uint32_t startTime = HAL_GetTick();
			while (HAL_GetTick() - startTime < TIMEOUT_MS) {
				// Check if a message is received
				if (CANSPI_Receive(&rxMessage)) {
					// Print received message
//					printf("Received Message: Data = {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}\n\r",
//							rxMessage.frame.data0, rxMessage.frame.data1,
//							rxMessage.frame.data2, rxMessage.frame.data3,
//							rxMessage.frame.data4, rxMessage.frame.data5,
//							rxMessage.frame.data6, rxMessage.frame.data7);
					//Decode PID
					decodePID(rxMessage.frame.data2, rxMessage.frame.data3,
							rxMessage.frame.data4, rxMessage.frame.data5,
							rxMessage.frame.data6, rxMessage.frame.data7);
					responseReceived[i] = true; // Mark response as received
					receivedResponse = true;
					break;
				}
			}
		} else {
			printf("Failed to send request for PID: 0x%02X\n\r", dataSets[i][2]);
		}

		if (!receivedResponse) {
			printf("No OBD2 response received, assuming car is off.\n");
			currentState = CAR_OFF;
			break;
		}

		// Small delay between requests
		HAL_Delay(50);
	}

	prevTX = HAL_GetTick();  // Update previous transmit time

	// Check all data received before logging
	if (receivedResponse) {
		bool allResponsesReceived = true;
		for (int i = 0; i < sizeof(dataSets) / sizeof(dataSets[0]); i++) {
			if (!responseReceived[i]) {
				allResponsesReceived = false;
				break;
			}
		}

		// All responses received = logging
		if (allResponsesReceived) {
			logPID();
			// Reset array
			memset(responseReceived, 0, sizeof(responseReceived));
		}
	}
}

// Implementation of Sleep Mode
void enterSleepMode(void) {
    printf("Entering sleep mode...\n\r");

    // MCP2515 entering sleep mode
    if (!MCP2515_SetSleepMode()) {
        printf("Failed to set Sleep mode!\n\r");
    } else {
        printf("MCP2515 set to Sleep mode.\n\r");
    }

    printf("Exiting sleep mode...\n\r");
    lastWakeTime = HAL_GetTick();
}


// Implementation of logging function
void logPID(void) {
	static int count = 1;
	printf("%d,%lu,%d,%d,%d,%d,%d,%d\n\r", count++, HAL_GetTick(), currentState,
			manifold_absolute_pressure, engine_speed, vehicle_speed,
			intake_air_temperature, control_module_voltage);
}


// Implementation of decoding PID
void decodePID(uint8_t pid, uint8_t a, uint8_t b, uint8_t c, uint8_t d,
		uint8_t e) {
	printf(
			"Received PID: 0x%02X, A: 0x%02X, B: 0x%02X, C: 0x%02X, D: 0x%02X, E: 0x%02X\n", pid, a, b, c, d, e);

	switch (pid) {
	case PID_MANIFOLD_ABSOLUTE_PRESSURE:
		manifold_absolute_pressure = a;
		printf("MAP: %d\n", manifold_absolute_pressure);
		break;
	case PID_ENGINE_SPEED:
		engine_speed = ((a * 256 + b) / 4);
		printf("Engine Speed: %d RPM\n", engine_speed);
		break;
	case PID_VEHICLE_SPEED:
		vehicle_speed = a;
		printf("Vehicle Speed: %d km/h\n", vehicle_speed);
		break;
	case PID_INTAKE_AIR_TEMPERATURE:
		intake_air_temperature = (a - 40);
		printf("Intake Air Temperature: %d °C\n", intake_air_temperature);
		break;
	case PID_CONTROL_MODULE_VOLTAGE:
		control_module_voltage = (((256 * a) + b) / 1000.0); //
		printf("Control Module Voltage: %.d V\n", control_module_voltage);
		break;
	default:
		break;
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK2
			| RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

// Configure PA4 as GPIO output for CAN CS
	GPIO_InitStruct.Pin = GPIO_PIN_4;    // Specify PA4
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;  // No pull-up or pull-down resistor
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low frequency, suitable for CS
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
//  /* Configure CS pin (GPIOB, Pin 0) */
//  GPIO_InitStruct.Pin = GPIO_PIN_0;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief BSP Push Button callback
 * @param Button Specifies the pressed button
 * @retval None
 */
void BSP_PB_Callback(Button_TypeDef Button) {
	switch (Button) {
	case BUTTON_SW1:
		/* Change the period to 100 ms */
		delay = 100;
		break;
	case BUTTON_SW2:
		/* Change the period to 500 ms */
		delay = 500;
		break;
	case BUTTON_SW3:
		/* Change the period to 1000 ms */
		delay = 1000;
		break;
	default:
		break;
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
