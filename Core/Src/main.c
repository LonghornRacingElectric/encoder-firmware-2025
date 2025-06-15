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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "usb_vcp.h"
#include "dfu.h"
#include "timer.h"
#include "led.h"
#include "night_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define ADC_CONVERSION_FACTOR  0.000801220703f
#define FILTER_SIZE 10

float cos_buffer[FILTER_SIZE] = {0};
int buffer_index = 0;
int buffer_filled = 0;

#define PACKET_ID = 0x0F0; // apps voltages

#define SENDING_INTERVAL_IN_MS, DATA_LENGTH_IN_BYTES);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
// }
int32_t green = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  // if your packet is set to 0 ms interval, it will send immediately every time you call
  // the CAN_AddTxPacket method. Unlikely that you will need to do this.
  // so add this outside of your while loop unless your interval is 0ms, in which case yes add it inside the loop wherever

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
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  // HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  // HAL_TIM_Base_Start(&htim5);
  clock_init();
  //led_init(TIM1, &htim5, 3);
  led_init(TIM5, &htim5, 3); // replace TIM15 and &htim15 with your timer
  dfu_init(GPIOA, GPIO_PIN_15);
  lib_timer_init();
  float prevtime;
  float cos;
  float mag;
  float cos_max = -INFINITY;
  float cos_min =  INFINITY;
  float sin_max = -INFINITY;
  float sin_min =  INFINITY;
  float prev_cos = 0.0;
  float prev_sin = 0.0;

  NightCANInstance nightCan = CAN_new_instance();
  CAN_Init( &nightCan, &hcan1, 0, 0x7FF, 0, 0); // change this line to either

  //APPS CAN
  NightCANPacket apps_voltages_pkt = CAN_create_packet(APPS_VOLTAGES_ID, APPS_VOLTAGES_FREQ, APPS_VOLTAGES_DLC);
  CAN_AddTxPacket(&nightCan, &apps_voltages_pkt);

  //BPPS CAN
  NightCANPacket bpps_voltages_pkt = CAN_create_packet(BPPS_VOLTAGES_ID, BPPS_VOLTAGES_FREQ, BPPS_VOLTAGES_DLC);
  CAN_AddTxPacket(&nightCan, &bpps_voltages_pkt);

  //BSE CAN
  NightCANPacket bse_voltages_pkt = CAN_create_packet(BSE_VOLTAGES_ID, BSE_VOLTAGES_FREQ, BSE_VOLTAGES_DLC);
  CAN_AddTxPacket(&nightCan, &bse_voltages_pkt);

//Brake CAN
  // NightCANPacket bspd_brake_pedal_pkt = CAN_create_packet(BRAKE_PEDAL_ID, BRAKE_PEDAL_FREQ, BRAKE_PEDAL_DLC);
  // CAN_AddTxPacket(&BRAKE_PEDAL, &bspd_brake_pedal_pkt);

//Steering Angle CAN
  NightCANPacket steering_angle_pkt = CAN_create_packet(RACK_STEERING_ID, RACK_STEERING_FREQ, RACK_STEERING_DLC);
  CAN_AddTxPacket(&nightCan, &steering_angle_pkt);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    receive_periodic();
    uint32_t curtime = lib_timer_delta_ms();


    CAN_periodic(&nightCan);
    // float apps1 = adc_getApps1();


    //usb_printf((char)lib_timer_delta_ms());
     led_rainbow(curtime / 1000.0f);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    float offset = 3.3f * 0.5f;

    float raw_cos = adc_getCos();
    float raw_sin = adc_getSin();

    // float filtered_cos = raw_cos;
    // float filtered_sin = raw_sin;

    //filtering
    float h = ((float)curtime - prevtime)/1000.0f;
    float timeconst = 0.050f;
    float alpha = h/(h+timeconst);
    float filtered_cos = raw_cos * alpha + prev_cos * (1-alpha);
    float filtered_sin = raw_sin * alpha + prev_sin * (1-alpha);
    prevtime = curtime;
    prev_cos = filtered_cos;
    prev_sin = filtered_sin;



    //calibration

    if (filtered_cos < cos_min) {
      cos_min = filtered_cos;
    }
    if (filtered_cos > cos_max) {
      cos_max = filtered_cos;
    }

    if (filtered_sin < sin_min) {
      sin_min = filtered_sin;
    }
    if (filtered_sin > sin_max) {
      sin_max = filtered_sin;
    }

    float amplitude_cos =  (cos_max - cos_min) / 2.0;
    float amplitude_sin =  (sin_max - sin_min) / 2.0;

    filtered_cos = (filtered_cos - offset) / amplitude_cos;
    filtered_sin = (filtered_sin - offset) / amplitude_sin;



    //calculating angle

    float angle = atan2(filtered_sin, filtered_cos);
    if (angle < 0)
      angle += 2.0f * M_PI;
    angle = angle * 180.0f / M_PI;



    float raw_mag = raw_sin*raw_sin + raw_cos*raw_cos;
    raw_mag = sqrt(raw_mag);

    // CAN_writeFloat(RACK_STEERING_STEERING_COLUMN_ANGLE_TYPE, &bspd_brake_pedal_pkt, BRAKE_PEDAL_BRAKE_PEDAL_TRAVEL_BYTE, bspd_brake, BRAKE_PEDAL_BRAKE_PEDAL_TRAVEL_PREC);

  // usb_printf("raw_mag = %f\n", raw_mag);
    // usb_printf("Cos: %0.3f, Sin: %0.3f, Angle: %0.3f\n", filtered_cos, filtered_sin, angle);

    float red = fabs(cos);   // Use absolute value to keep red positive

    // Make green more responsive to small changes in cos
    float green = 1.0f + sin(cos * M_PI);  // Varies between 0-2 based on cos

    // Add blue component that varies differently than red and green
    float blue = 2.0f - fmod(cos * 5.0f, 2.0f);  // Creates a different pattern

//     // Constrain values to valid LED range (typically 0-1 or 0-255 depending on your system)
     if (cos >= 1.5f && cos <= 2.5f) {
       red = fmin(fmax(red, 0.0f), 1.0f);
       green = fmin(fmax(green, 0.0f), 1.0f);
       blue = fmin(fmax(blue, 0.0f), 1.0f);
     }
//
//
    // Update LED with new values and add small delay
    led_set(red, green, blue);
    HAL_Delay(2);  // Shorter delay for more responsive updates



//other signals
    float apps1 = adc_getApps1(); //verified
    // usb_printf("apps1 %f\n", apps1);
    CAN_writeFloat(APPS_VOLTAGES_APPS1_VOLTAGE_TYPE, &apps_voltages_pkt, APPS_VOLTAGES_APPS1_VOLTAGE_BYTE, apps1, APPS_VOLTAGES_APPS1_VOLTAGE_PREC);

    float apps2 = adc_getApps2();
    // usb_printf("apps2 %f\n", apps2);
    CAN_writeFloat(APPS_VOLTAGES_APPS2_VOLTAGE_TYPE, &apps_voltages_pkt, APPS_VOLTAGES_APPS2_VOLTAGE_BYTE, apps2, APPS_VOLTAGES_APPS2_VOLTAGE_PREC);

    float bpps1 = adc_getBpps1();
    // usb_printf("bpps1 %f\n", bpps1);
    // CAN_writeFloat(BPPS_VOLTAGES_BPPS1_TRAVEL_TYPE, &bpps1_voltages_pkt, BPPS_VOLTAGES_BPPS1_TRAVEL_BYTE, bpps1, BPPS_VOLTAGES_BPPS1_TRAVEL_PREC);

    float bpps2 = adc_getBpps2();
    // usb_printf("bpps2 %f\n", bpps2); // works just like printf, use like printf
    // CAN_writeFloat(BPPS_VOLTAGES_BPPS2_TRAVEL_TYPE, &bpps2_voltages_pkt, BPPS_VOLTAGES_BPPS2_TRAVEL_BYTE, bpps2, BPPS_VOLTAGES_BPPS2_TRAVEL_PREC);
    //
    float bse1 = adc_getBse1();
    bse1 = ((bse1-0.5f)/4.0f) * 3000.0f;
    // usb_printf("bse1 %f\n", bse1);
    // CAN_writeFloat(BSE_VOLTAGES_BSE_FRONT_VOLTAGE_TYPE, &bse1_voltages_pkt, BSE_VOLTAGES_BSE_FRONT_VOLTAGE_BYTE, bse1, BSE_VOLTAGES_BSE_FRONT_VOLTAGE_PREC);


    float bse2 = adc_getBse2();
    bse2 = ((bse2-0.5f)/4.0f )* 3000.0f;
    // usb_printf("bse2 %f\n", bse2);
    usb_printf("bse1 = %f, bse2 = %f", bse1, bse2);
    // CAN_writeFloat(BSE_VOLTAGES_BSE_REAR_VOLTAGE_TYPE, &bse2_voltages_pkt, BSE_VOLTAGES_BSE_REAR_VOLTAGE_BYTE, bse2, BSE_VOLTAGES_BSE_REAR_VOLTAGE_PREC);

    float bspd_brake = adc_getBSPD_Brake_Analog();
    usb_printf("bspd_brake %f\n", bspd_brake);
    // CAN_writeFloat(BRAKE_PEDAL_BRAKE_PEDAL_TRAVEL_TYPE, &bspd_brake_pedal_pkt, BRAKE_PEDAL_BRAKE_PEDAL_TRAVEL_BYTE, bspd_brake, BRAKE_PEDAL_BRAKE_PEDAL_TRAVEL_PREC);


    // float steer_vgmr = adc_getSteerVGMR();

    //PRINT EVERYTHING
  usb_printf("bse1 = %f, bse2 = %f, bspd_brake = %f, apps1 = %f, apps2 = %f, bpps1 = %f, bpps2 = %f", bse1, bse2, bspd_brake, apps1, apps2, bpps1, bpps2);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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
  // __disable_irq();
  while (1)
  {
    led_set(255, 0, 0);
    receive_periodic();
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
