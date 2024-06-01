
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

  #include "cmsis_os.h"
  #include "usart.h"
  #include "gpio.h"
  #include "semphr.h"
  #include "task.h"
  #include "event_groups.h"
  #include "FreeRTOS.h"
  #include "dma.h"
  #include "tim.h"
  #include <string.h>
  #include "led.h"


  //#include "chat_helper_functions.h"
  #include <ctype.h>
  #include <stdio.h>
#define SET_COLOR 1
#define SET_BRIGHTNESS 2
TickType_t shortTimeOut = 50;
  QueueHandle_t uart_rx_queue;
  SemaphoreHandle_t xProcesses_packet_sema;
  TaskHandle_t task1Handler;
  TaskHandle_t task2Handler;
  TaskHandle_t task3Handler;
  TaskHandle_t task4Handler;
  #define UART_RX_QUEUE_LENGTH 4
  uint8_t datasentflag = 0;
  uint8_t uart_rx_char = 0;
  uint8_t Packet[5] = {0};
  uint8_t Packet_Len = 0;

  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */

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

  /* USER CODE END PV */

  /* Private function prototypes -----------------------------------------------*/
  void SystemClock_Config(void);


  /* USER CODE BEGIN PFP */

  void Task11();
  void Tasky();
  void GPIO_Init(void);
  uint8_t rxokay = 0;

  //TIM_HandleTypeDef htim1;
  //DMA_HandleTypeDef hdma_tim1_ch1;

  /* USER CODE END PFP */

  /* Private user code ---------------------------------------------------------*/
  /* USER CODE BEGIN 0 */

  /* USER CODE END 0 */

  /**
    * @brief  The application entry point.
    * @retval int
    */
  int main(void)
  {
    /* USER CODE BEGIN 1 */
  	BaseType_t retVal;	// used for checking task creation
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
     MX_USART1_UART_Init();
     GPIO_Init();
     MX_TIM1_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Init scheduler */
   // osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
    //MX_FREERTOS_Init();

    /* Start scheduler */
    //osKernelStart();


     retVal = xTaskCreate(Task11, "task1", configMINIMAL_STACK_SIZE *3 , NULL, tskIDLE_PRIORITY + 1, &task1Handler);
       if (retVal != pdPASS) {

    	   return 1;
       }	// c
       retVal = xTaskCreate(Tasky, "task2", configMINIMAL_STACK_SIZE  , NULL, tskIDLE_PRIORITY + 1, &task2Handler);
           if (retVal != pdPASS) {

        	   return 1;
           }	// c

       xProcesses_packet_sema = xSemaphoreCreateBinary();

       vTaskStartScheduler();

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
  }

  void Task11()
  {


  	RGB_Value current_color = {0};
  	uint8_t brightness = 100;
  	uint8_t packet[4] = { 0};

  	//uart_rx_queue = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(char));
  	 /// __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
  	  //  __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);


  	set_led_color(0,0,0,1);
  	for(;;)
  	{
  	 HAL_UART_Receive_IT(&huart1, &uart_rx_char, sizeof(uart_rx_char));

  	 if(xSemaphoreTake(xProcesses_packet_sema, shortTimeOut) == pdTRUE)
  	 {
  		 switch(Packet[0])
  		 {
  		 	 case SET_COLOR:
  		 		 current_color.red = Packet[1]; current_color.green = Packet[2]; current_color.blue = Packet[3];
  		 	     set_led_color(current_color.red ,current_color.green,current_color.blue,brightness);
  		 	     break;

  		 	 case SET_BRIGHTNESS:
  		 		brightness = Packet[1];
  		 	     set_led_color(current_color.red ,current_color.green,current_color.blue,brightness);
  		 	     break;

  		 	 default:
  		 		 break;
  		 }

  		 Packet_Len = 0;
  	  }
  	 else
  	  	 HAL_UART_Receive_IT(&huart1, &uart_rx_char, sizeof(uart_rx_char));

  }
  }
  void Tasky()
  {
  	uint32_t i = 0;


  	for(;;)
  	{
  		//requests 1 char from usart async
  		vTaskDelay(1000 );
  		i++;
  	}
  }

  void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
  {
  	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
  	datasentflag=1;
  }
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
  {
  	 if (huart == &huart1)
  	     {
  		 	 if(Packet_Len<4)
  		 	 {
  		   	   Packet[Packet_Len] = uart_rx_char;
  		   	   Packet_Len++;

  		 	 }
  		 	 else
  		 	 {
  		 		 BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  		 		 xSemaphoreGiveFromISR(xProcesses_packet_sema,xHigherPriorityTaskWoken);
  		 	 }

  	    }
  }

  void GPIO_Init(void)
  {
 	    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  	    // Configure PA5 (LD2) as an output
  	    GPIOA->MODER &= ~(GPIO_MODER_MODE5_Msk);
  	    GPIOA->MODER |= GPIO_MODER_MODE5_0; // Output mode
  	    //random pin to toggle for counting task
  	    GPIOA->MODER &= ~(GPIO_MODER_MODE0_Msk);
  	   	GPIOA->MODER |= GPIO_MODER_MODE0_0; // Output mode
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM15 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM15) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
