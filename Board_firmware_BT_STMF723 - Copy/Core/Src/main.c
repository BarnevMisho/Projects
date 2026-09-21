/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for Tof VL53L8CX sensor boards and MCU main board
  * @author         : Mihail Barnev
  * @organization   : ETH Zurich
  * @project        : Part of Bachelor's Thesis
  * @date           : 07/06/2026
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "vl53l8cx_api.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ToF_ROWS_COLS 8
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;

/* USER CODE BEGIN PV */
int status;
uint8_t isAlive, isReady;
VL53L8CX_Configuration tof4_1, tof4_2, tof4_3, tof1_1, tof1_2, tof1_3;
VL53L8CX_ResultsData Results;

// --- INTERRUPT FLAGS (SPI_4 ONLY) ---
volatile uint8_t data_ready_4_1 = 0;
volatile uint8_t data_ready_4_2 = 0;
volatile uint8_t data_ready_4_3 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    CDC_Transmit_HS((uint8_t*)ptr, len);
    HAL_Delay(1); // Small buffer delay
    return len;
}

// hardware interrupt callback function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT_4_1_Pin) {data_ready_4_1 = 1;}
    if(GPIO_Pin == INT_4_2_Pin) {data_ready_4_2 = 1;}
    if(GPIO_Pin == INT_4_3_Pin) {data_ready_4_3 = 1;}
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_SPI4_Init();
  MX_USB_DEVICE_Init();

  	/* USER CODE BEGIN 2 */
  	setbuf(stdout, NULL);
  	HAL_Delay(10000); // 10 seconds to open PuTTY

	printf("\r\n\r\n--- Starting Custom Board Test (6-Sensor Array) ---\r\n");

	// ==========================================
	// SENSOR 4_1 CONFIGURATION (SPI 4)
	// ==========================================
	tof4_1.platform.hspi = &hspi4;
	tof4_1.platform.ncs_gpio_port = NCS_4_1_GPIO_Port;
	tof4_1.platform.ncs_pin = NCS_4_1_Pin;

	vl53l8cx_set_power_mode(&tof4_1, VL53L8CX_POWER_MODE_DEEP_SLEEP);
	VL53L8CX_Reset_Sensor(&tof4_1.platform);

	status = vl53l8cx_is_alive(&tof4_1, &isAlive);
	if(!isAlive){
	printf("Sensor_4_1 VL53L8CX with SPI communication not detected, error : %d\r\n", status);
	} else {
	printf("Sensor_4_1 initializing, please wait few seconds\r\n");
	status = vl53l8cx_init(&tof4_1);
	if(status){
	printf("Sensor_4_1 Init failed with status %d\r\n", status);
	} else {

	status = vl53l8cx_set_resolution(&tof4_1, VL53L8CX_RESOLUTION_8X8); // two choices 4x4 & 8x8
	status = vl53l8cx_set_ranging_frequency_hz(&tof4_1, 15); // 15Hz Frequency - maximum allowed for 8x8 resolution
	status = vl53l8cx_set_ranging_mode(&tof4_1, VL53L8CX_RANGING_MODE_CONTINUOUS); // Continuous Ranging Mode - better than autonomous for robodog
	// Continuous mode is advised for fast ranging measurements or high performances.
	// Autonomous mode is advised for low power applications.
	status = vl53l8cx_set_target_order(&tof4_1, VL53L8CX_TARGET_ORDER_CLOSEST); // Closest Target - because avoiding imminent collisions is priority
	// The VL53L8CX can measure several targets per zone  two options:
	// Closest: The closest target is the first reported
	// Strongest: The strongest target is the first reported
	status = vl53l8cx_set_sharpener_percent(&tof4_1, 20); // Sharpener to 20% - most optimal value to get the real scene
	// The sharpener is used to remove some or all of the signal caused by a veiling glare; 0% romeves nothing, 99% removes all glare
	status = vl53l8cx_set_VHV_repeat_count(&tof4_1, 450); // Periodic Temperature Compensation every 30 sec
	// The ranging performance is affected by temperature variations, so we want to be aligned with the new temperature

	printf("Sensor_4_1 Ranging starts\r\n");
	status = vl53l8cx_start_ranging(&tof4_1);
	}
	}
	printf("Waiting for power to stabilize...\r\n");
	HAL_Delay(250);

	// ==========================================
	// SENSOR 4_2 CONFIGURATION (SPI 4)
	// ==========================================
	tof4_2.platform.hspi = &hspi4;
	tof4_2.platform.ncs_gpio_port = NCS_4_2_GPIO_Port;
	tof4_2.platform.ncs_pin = NCS_4_2_Pin;

	vl53l8cx_set_power_mode(&tof4_2, VL53L8CX_POWER_MODE_DEEP_SLEEP);
	VL53L8CX_Reset_Sensor(&tof4_2.platform);

	status = vl53l8cx_is_alive(&tof4_2, &isAlive);
	if(!isAlive){
	printf("Sensor_4_2 VL53L8CX with SPI communication not detected, error : %d\r\n", status);
	} else {
	printf("Sensor_4_2 initializing, please wait few seconds\r\n");
	status = vl53l8cx_init(&tof4_2);
	if(status){
	printf("Sensor_4_2 Init failed with status %d\r\n", status);
	} else {
	status = vl53l8cx_set_resolution(&tof4_2, VL53L8CX_RESOLUTION_8X8);
	status = vl53l8cx_set_ranging_frequency_hz(&tof4_2, 15);
	status = vl53l8cx_set_ranging_mode(&tof4_2, VL53L8CX_RANGING_MODE_CONTINUOUS);
	status = vl53l8cx_set_target_order(&tof4_2, VL53L8CX_TARGET_ORDER_CLOSEST);
	status = vl53l8cx_set_sharpener_percent(&tof4_2, 20);
	status = vl53l8cx_set_VHV_repeat_count(&tof4_2, 450);

	printf("Sensor_4_2 Ranging starts\r\n");
	status = vl53l8cx_start_ranging(&tof4_2);
	}
	}
	printf("Waiting for power to stabilize...\r\n");
	HAL_Delay(250);

	// ==========================================
	// SENSOR 4_3 CONFIGURATION (SPI 4)
	// ==========================================
	tof4_3.platform.hspi = &hspi4;
	tof4_3.platform.ncs_gpio_port = NCS_4_3_GPIO_Port;
	tof4_3.platform.ncs_pin = NCS_4_3_Pin;

	vl53l8cx_set_power_mode(&tof4_3, VL53L8CX_POWER_MODE_DEEP_SLEEP);
	VL53L8CX_Reset_Sensor(&tof4_3.platform);

	status = vl53l8cx_is_alive(&tof4_3, &isAlive);
	if(!isAlive){
	printf("Sensor_4_3 VL53L8CX with SPI communication not detected, error : %d\r\n", status);
	} else {
	printf("Sensor_4_3 initializing, please wait few seconds\r\n");
	status = vl53l8cx_init(&tof4_3);
	if(status){
	printf("Sensor_4_3 Init failed with status %d\r\n", status);
	} else {
	status = vl53l8cx_set_resolution(&tof4_3, VL53L8CX_RESOLUTION_8X8);
	status = vl53l8cx_set_ranging_frequency_hz(&tof4_3, 15);
	status = vl53l8cx_set_ranging_mode(&tof4_3, VL53L8CX_RANGING_MODE_CONTINUOUS);
	status = vl53l8cx_set_target_order(&tof4_3, VL53L8CX_TARGET_ORDER_CLOSEST);
	status = vl53l8cx_set_sharpener_percent(&tof4_3, 20);
	status = vl53l8cx_set_VHV_repeat_count(&tof4_3, 450);

	printf("Sensor_4_3 Ranging starts\r\n");
	status = vl53l8cx_start_ranging(&tof4_3);
	}
	}
	printf("Waiting for power to stabilize...\r\n");
	HAL_Delay(250);

	// ==========================================
	// SENSOR 1_1 CONFIGURATION (SPI 1)
	// ==========================================
	tof1_1.platform.hspi = &hspi1; // Using SPI1
	tof1_1.platform.ncs_gpio_port = NCS_1_1_GPIO_Port;
	tof1_1.platform.ncs_pin = NCS_1_1_Pin;

	vl53l8cx_set_power_mode(&tof1_1, VL53L8CX_POWER_MODE_DEEP_SLEEP);
	VL53L8CX_Reset_Sensor(&tof1_1.platform);

	status = vl53l8cx_is_alive(&tof1_1, &isAlive);
	if(!isAlive){
	printf("Sensor_1_1 VL53L8CX with SPI communication not detected, error : %d\r\n", status);
	} else {
	printf("Sensor_1_1 initializing, please wait few seconds\r\n");
	status = vl53l8cx_init(&tof1_1);
	if(status){
	printf("Sensor_1_1 Init failed with status %d\r\n", status);
	} else {
	status = vl53l8cx_set_resolution(&tof1_1, VL53L8CX_RESOLUTION_8X8);
	status = vl53l8cx_set_ranging_frequency_hz(&tof1_1, 15);
	status = vl53l8cx_set_ranging_mode(&tof1_1, VL53L8CX_RANGING_MODE_CONTINUOUS);
	status = vl53l8cx_set_target_order(&tof1_1, VL53L8CX_TARGET_ORDER_CLOSEST);
	status = vl53l8cx_set_sharpener_percent(&tof1_1, 20);
	status = vl53l8cx_set_VHV_repeat_count(&tof1_1, 450);

	printf("Sensor_1_1 Ranging starts\r\n");
	status = vl53l8cx_start_ranging(&tof1_1);
	}
	}
	printf("Waiting for power to stabilize...\r\n");
	HAL_Delay(250);

	// ==========================================
	// SENSOR 1_2 CONFIGURATION (SPI 1)
	// ==========================================
	tof1_2.platform.hspi = &hspi1;
	tof1_2.platform.ncs_gpio_port = NCS_1_2_GPIO_Port;
	tof1_2.platform.ncs_pin = NCS_1_2_Pin;

	vl53l8cx_set_power_mode(&tof1_2, VL53L8CX_POWER_MODE_DEEP_SLEEP);
	VL53L8CX_Reset_Sensor(&tof1_2.platform);

	status = vl53l8cx_is_alive(&tof1_2, &isAlive);
	if(!isAlive){
	printf("Sensor_1_2 VL53L8CX with SPI communication not detected, error : %d\r\n", status);
	} else {
	printf("Sensor_1_2 initializing, please wait few seconds\r\n");
	status = vl53l8cx_init(&tof1_2);
	if(status){
	printf("Sensor_1_2 Init failed with status %d\r\n", status);
	} else {
	status = vl53l8cx_set_resolution(&tof1_2, VL53L8CX_RESOLUTION_8X8);
	status = vl53l8cx_set_ranging_frequency_hz(&tof1_2, 15);
	status = vl53l8cx_set_ranging_mode(&tof1_2, VL53L8CX_RANGING_MODE_CONTINUOUS);
	status = vl53l8cx_set_target_order(&tof1_2, VL53L8CX_TARGET_ORDER_CLOSEST);
	status = vl53l8cx_set_sharpener_percent(&tof1_2, 20);
	status = vl53l8cx_set_VHV_repeat_count(&tof1_2, 450);

	printf("Sensor_1_2 Ranging starts\r\n");
	status = vl53l8cx_start_ranging(&tof1_2);
	}
	}
	printf("Waiting for power to stabilize...\r\n");
	HAL_Delay(250);

	// ==========================================
	// SENSOR 1_3 CONFIGURATION (SPI 1)
	// ==========================================
	tof1_3.platform.hspi = &hspi1;
	tof1_3.platform.ncs_gpio_port = NCS_1_3_GPIO_Port;
	tof1_3.platform.ncs_pin = NCS_1_3_Pin;

	vl53l8cx_set_power_mode(&tof1_3, VL53L8CX_POWER_MODE_DEEP_SLEEP);
	VL53L8CX_Reset_Sensor(&tof1_3.platform);

	status = vl53l8cx_is_alive(&tof1_3, &isAlive);
	if(!isAlive){
	printf("Sensor_1_3 VL53L8CX with SPI communication not detected, error : %d\r\n", status);
	} else {
	printf("Sensor_1_3 initializing, please wait few seconds\r\n");
	status = vl53l8cx_init(&tof1_3);
	if(status){
	printf("Sensor_1_3 Init failed with status %d\r\n", status);
	} else {
	status = vl53l8cx_set_resolution(&tof1_3, VL53L8CX_RESOLUTION_8X8);
	status = vl53l8cx_set_ranging_frequency_hz(&tof1_3, 15);
	status = vl53l8cx_set_ranging_mode(&tof1_3, VL53L8CX_RANGING_MODE_CONTINUOUS);
	status = vl53l8cx_set_target_order(&tof1_3, VL53L8CX_TARGET_ORDER_CLOSEST);
	status = vl53l8cx_set_sharpener_percent(&tof1_3, 20);
	status = vl53l8cx_set_VHV_repeat_count(&tof1_3, 450);

	printf("Sensor_1_3 Ranging starts\r\n");
	status = vl53l8cx_start_ranging(&tof1_3);
	}
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 /* =========================================================================
	 * MULTI-TARGET DETECTION & ROBODOG FILTERING EXPLANATION
	 * =========================================================================
	 * The VL53L8CX shoots a 45-degree cone of light divided into an 8x8 grid.
	 * Because a single zone acts as a cone, it can hit the edge of an object (Target 1)
	 * and the wall behind that object (Target 2) simultaneously.
	 * t1_idx = The closest object detected in this specific zone.
	 * t2_idx = The background object detected in this specific zone.
	 * NOTE ON TARGET 2: If the sensor looks at a flat wall, all light bounces back
	 * at the same time (nb_target_detected == 1 and not 2). Target 2 will print as 'X' because
	 * there is no background object. 'X' for T2 means "this is a solid, flat surface."
	 * 'X' means we don't need the measurement because of TARGET STATUS FILTER:
	 * We must filter out noise so the robodog does not hallucinate "ghost" walls.
	 * - 5, 6, 9, 10 : 100% Valid distance. Keep.
	 * - 12, 13      : Blurry or secondary target, but an object physically exists. Keep.
	 * - 0-4, 7-8, 11: Sensor math failed or signal too weak. Discard (Print 'X').
	 * - 255         : Absolutely no object detected. Discard (Print 'X').
	 * ========================================================================= */


	// =========================================================================
	// SPI_4 SENSORS (INTERRUPT DRIVEN)
	// =========================================================================

	// --- CHECK SENSOR 4_1 (INTERRUPT) ---
	if(data_ready_4_1)
	{
		data_ready_4_1 = 0;
		vl53l8cx_get_ranging_data(&tof4_1, &Results);

		printf("\r\n--- SENSOR 4_1 (Frame %d) | Format: [T1 / T2] ---\r\n", tof4_1.streamcount);
		for(uint8_t i = 0; i < ToF_ROWS_COLS; i++)
		{
			printf("|");
			for(uint8_t j = 0; j < ToF_ROWS_COLS; j++)
			{
				int zone_idx = i * 8 + j;
				int t1_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 0;
				int t2_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 1;
				int t1_valid = 0;
				int t2_valid = 0;

				if(Results.nb_target_detected[zone_idx] > 0 &&
				  (Results.target_status[t1_idx] == 5 || Results.target_status[t1_idx] == 6 ||
				   Results.target_status[t1_idx] == 9 || Results.target_status[t1_idx] == 10 ||
				   Results.target_status[t1_idx] == 12 || Results.target_status[t1_idx] == 13)) {
					t1_valid = 1;
				}
				if(Results.nb_target_detected[zone_idx] > 1 &&
				  (Results.target_status[t2_idx] == 5 || Results.target_status[t2_idx] == 6 ||
				   Results.target_status[t2_idx] == 9 || Results.target_status[t2_idx] == 10 ||
				   Results.target_status[t2_idx] == 12 || Results.target_status[t2_idx] == 13)) {
					t2_valid = 1;
				}

				if (t1_valid && t2_valid) {
					printf(" %4dmm/%4dmm |", Results.distance_mm[t1_idx], Results.distance_mm[t2_idx]);
				} else if (t1_valid && !t2_valid) {
					printf(" %4dmm/   X   |", Results.distance_mm[t1_idx]);
				} else if (!t1_valid && t2_valid) {
					printf("    X  /%4dmm |", Results.distance_mm[t2_idx]);
				} else {
					printf("    X  /   X   |");
				}
			}
			printf("\r\n");
		}
	}

	// --- CHECK SENSOR 4_2 (INTERRUPT) ---
	if(data_ready_4_2)
	{
		data_ready_4_2 = 0;
		vl53l8cx_get_ranging_data(&tof4_2, &Results);

		printf("\r\n--- SENSOR 4_2 (Frame %d) | Format: [T1 / T2] ---\r\n", tof4_2.streamcount);
		for(uint8_t i = 0; i < ToF_ROWS_COLS; i++)
		{
			printf("|");
			for(uint8_t j = 0; j < ToF_ROWS_COLS; j++)
			{
				int zone_idx = i * 8 + j;
				int t1_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 0;
				int t2_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 1;
				int t1_valid = 0;
				int t2_valid = 0;

				if(Results.nb_target_detected[zone_idx] > 0 &&
				  (Results.target_status[t1_idx] == 5 || Results.target_status[t1_idx] == 6 ||
				   Results.target_status[t1_idx] == 9 || Results.target_status[t1_idx] == 10 ||
				   Results.target_status[t1_idx] == 12 || Results.target_status[t1_idx] == 13)) {
					t1_valid = 1;
				}
				if(Results.nb_target_detected[zone_idx] > 1 &&
				  (Results.target_status[t2_idx] == 5 || Results.target_status[t2_idx] == 6 ||
				   Results.target_status[t2_idx] == 9 || Results.target_status[t2_idx] == 10 ||
				   Results.target_status[t2_idx] == 12 || Results.target_status[t2_idx] == 13)) {
					t2_valid = 1;
				}

				if (t1_valid && t2_valid) {
					printf(" %4dmm/%4dmm |", Results.distance_mm[t1_idx], Results.distance_mm[t2_idx]);
				} else if (t1_valid && !t2_valid) {
					printf(" %4dmm/   X   |", Results.distance_mm[t1_idx]);
				} else if (!t1_valid && t2_valid) {
					printf("    X  /%4dmm |", Results.distance_mm[t2_idx]);
				} else {
					printf("    X  /   X   |");
				}
			}
			printf("\r\n");
		}
	}

	// --- CHECK SENSOR 4_3 (INTERRUPT) ---
	if(data_ready_4_3)
	{
		data_ready_4_3 = 0;
		vl53l8cx_get_ranging_data(&tof4_3, &Results);

		printf("\r\n--- SENSOR 4_3 (Frame %d) | Format: [T1 / T2] ---\r\n", tof4_3.streamcount);
		for(uint8_t i = 0; i < ToF_ROWS_COLS; i++)
		{
			printf("|");
			for(uint8_t j = 0; j < ToF_ROWS_COLS; j++)
			{
				int zone_idx = i * 8 + j;
				int t1_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 0;
				int t2_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 1;
				int t1_valid = 0;
				int t2_valid = 0;

				if(Results.nb_target_detected[zone_idx] > 0 &&
				  (Results.target_status[t1_idx] == 5 || Results.target_status[t1_idx] == 6 ||
				   Results.target_status[t1_idx] == 9 || Results.target_status[t1_idx] == 10 ||
				   Results.target_status[t1_idx] == 12 || Results.target_status[t1_idx] == 13)) {
					t1_valid = 1;
				}
				if(Results.nb_target_detected[zone_idx] > 1 &&
				  (Results.target_status[t2_idx] == 5 || Results.target_status[t2_idx] == 6 ||
				   Results.target_status[t2_idx] == 9 || Results.target_status[t2_idx] == 10 ||
				   Results.target_status[t2_idx] == 12 || Results.target_status[t2_idx] == 13)) {
					t2_valid = 1;
				}

				if (t1_valid && t2_valid) {
					printf(" %4dmm/%4dmm |", Results.distance_mm[t1_idx], Results.distance_mm[t2_idx]);
				} else if (t1_valid && !t2_valid) {
					printf(" %4dmm/   X   |", Results.distance_mm[t1_idx]);
				} else if (!t1_valid && t2_valid) {
					printf("    X  /%4dmm |", Results.distance_mm[t2_idx]);
				} else {
					printf("    X  /   X   |");
				}
			}
			printf("\r\n");
		}
	}

	// =========================================================================
	// SPI_1 SENSORS (POLLING DRIVEN)
	// =========================================================================

	// --- CHECK SENSOR 1_1 (POLLING) ---
	isReady = 0;
	vl53l8cx_check_data_ready(&tof1_1, &isReady);
	if(isReady)
	{
		vl53l8cx_get_ranging_data(&tof1_1, &Results);

		printf("\r\n--- SENSOR 1_1 (Frame %d) | Format: [T1 / T2] ---\r\n", tof1_1.streamcount);
		for(uint8_t i = 0; i < ToF_ROWS_COLS; i++)
		{
			printf("|");
			for(uint8_t j = 0; j < ToF_ROWS_COLS; j++)
			{
				int zone_idx = i * 8 + j;
				int t1_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 0;
				int t2_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 1;
				int t1_valid = 0;
				int t2_valid = 0;

				if(Results.nb_target_detected[zone_idx] > 0 &&
				  (Results.target_status[t1_idx] == 5 || Results.target_status[t1_idx] == 6 ||
				   Results.target_status[t1_idx] == 9 || Results.target_status[t1_idx] == 10 ||
				   Results.target_status[t1_idx] == 12 || Results.target_status[t1_idx] == 13)) {
					t1_valid = 1;
				}
				if(Results.nb_target_detected[zone_idx] > 1 &&
				  (Results.target_status[t2_idx] == 5 || Results.target_status[t2_idx] == 6 ||
				   Results.target_status[t2_idx] == 9 || Results.target_status[t2_idx] == 10 ||
				   Results.target_status[t2_idx] == 12 || Results.target_status[t2_idx] == 13)) {
					t2_valid = 1;
				}

				if (t1_valid && t2_valid) {
					printf(" %4dmm/%4dmm |", Results.distance_mm[t1_idx], Results.distance_mm[t2_idx]);
				} else if (t1_valid && !t2_valid) {
					printf(" %4dmm/   X   |", Results.distance_mm[t1_idx]);
				} else if (!t1_valid && t2_valid) {
					printf("    X  /%4dmm |", Results.distance_mm[t2_idx]);
				} else {
					printf("    X  /   X   |");
				}
			}
			printf("\r\n");
		}
	}

	// --- CHECK SENSOR 1_2 (POLLING) ---
	isReady = 0;
	vl53l8cx_check_data_ready(&tof1_2, &isReady);
	if(isReady)
	{
		vl53l8cx_get_ranging_data(&tof1_2, &Results);

		printf("\r\n--- SENSOR 1_2 (Frame %d) | Format: [T1 / T2] ---\r\n", tof1_2.streamcount);
		for(uint8_t i = 0; i < ToF_ROWS_COLS; i++)
		{
			printf("|");
			for(uint8_t j = 0; j < ToF_ROWS_COLS; j++)
			{
				int zone_idx = i * 8 + j;
				int t1_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 0;
				int t2_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 1;
				int t1_valid = 0;
				int t2_valid = 0;

				if(Results.nb_target_detected[zone_idx] > 0 &&
				  (Results.target_status[t1_idx] == 5 || Results.target_status[t1_idx] == 6 ||
				   Results.target_status[t1_idx] == 9 || Results.target_status[t1_idx] == 10 ||
				   Results.target_status[t1_idx] == 12 || Results.target_status[t1_idx] == 13)) {
					t1_valid = 1;
				}
				if(Results.nb_target_detected[zone_idx] > 1 &&
				  (Results.target_status[t2_idx] == 5 || Results.target_status[t2_idx] == 6 ||
				   Results.target_status[t2_idx] == 9 || Results.target_status[t2_idx] == 10 ||
				   Results.target_status[t2_idx] == 12 || Results.target_status[t2_idx] == 13)) {
					t2_valid = 1;
				}

				if (t1_valid && t2_valid) {
					printf(" %4dmm/%4dmm |", Results.distance_mm[t1_idx], Results.distance_mm[t2_idx]);
				} else if (t1_valid && !t2_valid) {
					printf(" %4dmm/   X   |", Results.distance_mm[t1_idx]);
				} else if (!t1_valid && t2_valid) {
					printf("    X  /%4dmm |", Results.distance_mm[t2_idx]);
				} else {
					printf("    X  /   X   |");
				}
			}
			printf("\r\n");
		}
	}

	// --- CHECK SENSOR 1_3 (POLLING) ---
	isReady = 0;
	vl53l8cx_check_data_ready(&tof1_3, &isReady);
	if(isReady)
	{
		vl53l8cx_get_ranging_data(&tof1_3, &Results);

		printf("\r\n--- SENSOR 1_3 (Frame %d) | Format: [T1 / T2] ---\r\n", tof1_3.streamcount);
		for(uint8_t i = 0; i < ToF_ROWS_COLS; i++)
		{
			printf("|");
			for(uint8_t j = 0; j < ToF_ROWS_COLS; j++)
			{
				int zone_idx = i * 8 + j;
				int t1_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 0;
				int t2_idx = (VL53L8CX_NB_TARGET_PER_ZONE * zone_idx) + 1;
				int t1_valid = 0;
				int t2_valid = 0;

				if(Results.nb_target_detected[zone_idx] > 0 &&
				  (Results.target_status[t1_idx] == 5 || Results.target_status[t1_idx] == 6 ||
				   Results.target_status[t1_idx] == 9 || Results.target_status[t1_idx] == 10 ||
				   Results.target_status[t1_idx] == 12 || Results.target_status[t1_idx] == 13)) {
					t1_valid = 1;
				}
				if(Results.nb_target_detected[zone_idx] > 1 &&
				  (Results.target_status[t2_idx] == 5 || Results.target_status[t2_idx] == 6 ||
				   Results.target_status[t2_idx] == 9 || Results.target_status[t2_idx] == 10 ||
				   Results.target_status[t2_idx] == 12 || Results.target_status[t2_idx] == 13)) {
					t2_valid = 1;
				}

				if (t1_valid && t2_valid) {
					printf(" %4dmm/%4dmm |", Results.distance_mm[t1_idx], Results.distance_mm[t2_idx]);
				} else if (t1_valid && !t2_valid) {
					printf(" %4dmm/   X   |", Results.distance_mm[t1_idx]);
				} else if (!t1_valid && t2_valid) {
					printf("    X  /%4dmm |", Results.distance_mm[t2_idx]);
				} else {
					printf("    X  /   X   |");
				}
			}
			printf("\r\n");
		}
	}

	HAL_Delay(10); // Tiny delay to prevent polling from melting the SPI_1 bus
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, NCS_4_2_Pin|NCS_4_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NCS_1_1_GPIO_Port, NCS_1_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NCS_1_2_GPIO_Port, NCS_1_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NCS_1_3_Pin|NCS_4_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : NCS_4_2_Pin NCS_4_1_Pin */
  GPIO_InitStruct.Pin = NCS_4_2_Pin|NCS_4_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_4_2_Pin INT_4_1_Pin */
  GPIO_InitStruct.Pin = INT_4_2_Pin|INT_4_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : NCS_1_1_Pin */
  GPIO_InitStruct.Pin = NCS_1_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NCS_1_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NCS_1_2_Pin */
  GPIO_InitStruct.Pin = NCS_1_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NCS_1_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NCS_1_3_Pin NCS_4_3_Pin */
  GPIO_InitStruct.Pin = NCS_1_3_Pin|NCS_4_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_4_3_Pin */
  GPIO_InitStruct.Pin = INT_4_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_4_3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
