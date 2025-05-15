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


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    receive_periodic();
    uint32_t curtime = lib_timer_delta_ms();
    //usb_printf((char)lib_timer_delta_ms());
     led_rainbow(curtime / 1000.0f);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    float offset = 3.3f * 0.5f;

    float raw_cos = adc_getCos();
    float raw_sin = adc_getSin();

    //filtering
    float h = (float)curtime - prevtime;
    float timeconst = 0.050f;
    float alpha = h/(h+timeconst);
    raw_cos = raw_cos * alpha + prev_cos * (1-alpha);
    raw_sin = raw_sin * alpha + prev_sin * (1-alpha);
    prevtime = curtime;
    if (prev_cos != raw_cos)
      prev_cos = raw_cos;
    if (prev_sin != raw_sin)
      prev_sin = raw_sin;



    //calibration

    if (raw_cos < cos_min) {
      cos_min = raw_cos;
    }
    if (raw_cos > cos_max) {
      cos_max = raw_cos;
    }

    if (raw_sin < sin_min) {
      sin_min = raw_sin;
    }
    if (raw_sin > sin_max) {
      sin_max = raw_sin;
    }

    float amplitude_cos =  (cos_max - cos_min) / 2.0;
    float amplitude_sin =  (sin_max - sin_min) / 2.0;

    raw_cos = (raw_cos - offset) / amplitude_cos;
    raw_sin = raw_sin - offset / amplitude_sin;



    //calculating angle

    float angle = atan2(raw_sin, raw_cos);
    if (angle < 0)
      angle += 2.0f * M_PI;
    angle = angle * 180.0f / M_PI;



    // float raw_mag = raw_sin*raw_sin + raw_cos*raw_cos;
    // raw_mag = sqrt(raw_mag);




    // Print raw and filtered values for comparison
    // if (cos > 1.0f && cos < 2.0f) {
      // usb_printf("mag: %0.3f, filtered: %0.3f\n", adc_getCos(), adc_getSin());
    usb_printf("Cos: %0.3f, Sin: %0.3f, Angle: %0.3f\n", adc_getCos(), adc_getSin(), angle);

    // }
    // float apps1 = adc_getApps1() / 3.3f; //verified
    // usb_printf("apps1 %f\n", apps1); // works just like printf, use like printf
    // float bpps1 = adc_getBpps1() / 3.3f;
    // usb_printf("bpps1 %f\n", bpps1); // works just like printf, use like printf
    // float sin = adc_getSin_N() ;// / 3.3f - 0.5f;

    // if (sin > 1.0f && sin < 2.0f) {
    //   usb_printf("sin: %f\n", sin); // probably cosine actually
    // }
//claude wrote this
    // float cos = adc_getCosN();
    //
    // // Map the cosine value to a wider range for more visible color changes
    // float amplified_cos = cos * 3.5f;  // Amplify small changes
    float red = fabs(cos);   // Use absolute value to keep red positive

    // Make green more responsive to small changes in cos
    float green = 1.0f + sin(cos * M_PI);  // Varies between 0-2 based on cos

    // Add blue component that varies differently than red and green
    float blue = 2.0f - fmod(cos * 5.0f, 2.0f);  // Creates a different pattern

    // // Print debug info
    // usb_printf("cos: %0.3f, amplified: %0.3f\n",
    //            cos, amplified_cos);

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
//claude wrote this end



    // float cos = (adc_getCosN());
    //
    // if (cos > 1.0f && cos < 2.5f) {
    // green = green + (cos + 1);
    //   usb_printf("cos maybe: %0.3f\n", cos ); // probably sinN actually
    //   HAL_Delay(50);
    //   led_set(cos, green, 0);
    // }
    // usb_printf("cos: %f\n", cos);


    // float apps1 = adc_getApps1() / 3.3f; //verified
    // usb_printf("apps1 %f\n", apps1); // works just like printf, use like printf

    // float apps2 = adc_getApps2() / 3.3f;
    // usb_printf("apps2 %f\n", apps2); // works just like printf, use like printf

    // float bpps1 = adc_getBpps1() / 3.3f;
    // usb_printf("bpps1 %f\n", bpps1); // works just like printf, use like printf
    //
    // float bpps2 = adc_getBpps2() / 3.3f;
    // usb_printf("bpps2 %f\n", bpps2); // works just like printf, use like printf

    //
    // float bse1 = adc_getBse1() / 3.3f;
    // float bse2 = adc_getBse2() / 3.3f;
    //
    // float bspd_brake = adc_getBSPD_Brake_Analog() / 3.3f;
    // float steer_vgmr = adc_getSteerVGMR() / 3.3f;
    //
    // float sin = adc_getSin_N();// / 3.3f - 0.5f;
    // usb_printf("sin: %f\n", sin); //verified
    // float cos = adc_getCos() / 3.3f - 0.5f;
    // usb_printf("cos: %f\n", cos);
    //
    // float mag = sin*sin + cos*cos;
    // led_set(0, mag, 0);
    //
    // if(mag > 0.03f)
    // {
    //   float ang = atan2f(sin, cos) / M_PI / 2.0f + 0.5f;
    //   if(ang < 1.0f/3.0f) {
    //     ang = ang * 3.0f;
    //     led_set(1.0f-ang, ang, 0);
    //   } else if (ang < 2.0f/3.0f) {
    //     ang = ang * 3.0f - 1.0f;
    //     led_set(0, 1.0f-ang, ang);
    //   } else {
    //     ang = ang * 3.0f - 2.0f;
    //     led_set(ang, 0, 1.0f-ang);
    //   }
    // } else
    // {
    //   led_off();
    // }

    // float deltaTime = clock_getDeltaTime();
    // led_rainbow(deltaTime);
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
