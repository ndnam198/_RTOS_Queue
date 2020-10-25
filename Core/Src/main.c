/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
	Ví dụ này chỉ ra cách sử dụng queue, tận dụng queue để truyền dữ liệu liên tục với các giá trị 100 và 200 vào hàng đợi và đọc từ hàng đợi ngay khi hàng đợi có dữ liệu đồng thời in ra màn hình debug uart.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
/* USER CODE END Includes */
#include "queue.h"
#include "myLib.h"
#include "task.h"
#include "myRTOSaddons.h"
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
UART_HandleTypeDef huart2;

/* Definitions for blinkLed1 */
osThreadId_t blinkLed1Handle;
const osThreadAttr_t blinkLed1_attributes = {
    .name = "blinkLed1",
    .priority = (osPriority_t)osPriorityLow5,
    .stack_size = 32 * 4};
/* Definitions Sender Task*/
osThreadId_t SenderHandle1;
osThreadId_t SenderHandle2;
const osThreadAttr_t Sender_attributes = {
    .name = "Sender",
    .priority = (osPriority_t)osPriorityLow3,
    .stack_size = 128 * 4};
/* Definitions Receiver Task*/
osThreadId_t ReceiverHandle;
const osThreadAttr_t Receiver_attributes = {
    .name = "Receiver",
    .priority = (osPriority_t)osPriorityLow7, /* ReceiveTask is the highest priority task ==> Queue's always empty */
    .stack_size = 128 * 4};

/* USER CODE BEGIN PV */
osMessageQueueId_t queue_handle = NULL;
static uint32_t error_count = 0;
static uint32_t data_to_send[2] = {100, 200};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void haltCheckLedTask(void *argument);
void senderTask(void *argument);
void receiverTask(void *argument);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  offAllLed;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  queue_handle = osMessageQueueNew(5, sizeof(uint32_t), NULL);
  if (queue_handle == NULL)
  {
    error_count++;
    PRINTF("Failed to create new queue\r\n");
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* creation of blinkLed1 */
  blinkLed1Handle = osThreadNew(haltCheckLedTask, NULL, &blinkLed1_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* creation of Sender task*/
  SenderHandle1 = osThreadNew(senderTask, (void *)data_to_send, &Sender_attributes);
  if (SenderHandle1 == NULL)
  {
    PRINTF("Failed creating senderTask1 \r\n");
    error_count++;
  }
  SenderHandle2 = osThreadNew(senderTask, (void *)(data_to_send + 1), &Sender_attributes);
  if (SenderHandle2 == NULL)
  {
    PRINTF("Failed creating senderTask2 \r\n");
    error_count++;
  }
  /* creation of Receiver task */
  ReceiverHandle = osThreadNew(receiverTask, NULL, &Receiver_attributes);
  if (ReceiverHandle == NULL)
  {
    PRINTF("Failed creating receiverTask \r\n");
    error_count++;
  }
  /* add threads, ... */
  printVar(error_count);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  if (error_count == 0)
  {
    PRINTF("Start RTOS_Kernel\r\n");
    osKernelStart();
  }
  else
  {
    PRINTF("Failed to start RTOS_Kernel\r\n");
  }
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/*senderTask */
/**

*/
/* USER CODE END Header_haltCheckLedTask */
void senderTask(void *argument)
{
  TickType_t block_time = pdMS_TO_TICKS(1000);
  /* USER CODE BEGIN haltCheckLedTask */
  BaseType_t status;
  /* Get parameter into variable */
  uint32_t *ul_value_to_send;
  ul_value_to_send = (uint32_t *)argument; /* Pointer to data */
  /* Infinite loop */
  for (;;)
  {
    PRINT_IN_TASK("SenderTask execute\r\n");
    (*ul_value_to_send)++;
      status = xQueueSendToBack(queue_handle, (void *)ul_value_to_send, 0);
    if (status != pdPASS)
    {
      PRINT_IN_TASK("Failed to send to queue\r\n");
    }
    osDelay(block_time); /* Task execute every block_time */
  }
  /* USER CODE END haltCheckLedTask */
}

/*receiverTask */
/**

*/
/* USER CODE END Header_haltCheckLedTask */
void receiverTask(void *argument)
{
  const TickType_t block_time = pdMS_TO_TICKS(10000);
  BaseType_t status;
  static uint32_t sequence_id = 0;
  uint32_t receive_data;
  uint8_t nb_data_on_queue = 0;
  /* Infinite loop */
  for (;;)
  {
    newline;
    ++sequence_id;
    PRINT_VAR_IN_TASK(sequence_id);
    /* receiverTask has no specified block_time ==> whenever new data arrives on queue, receive_data will be unblocked  */
    nb_data_on_queue = (uint8_t)uxQueueMessagesWaiting(queue_handle); /* Returns the number of items that are currently held in a queue */
    if (nb_data_on_queue != 0)
    {
      PRINT_IN_TASK("Queue should have been empty\r\n");
    }

    status = xQueueReceive(queue_handle, &receive_data, block_time);
    if (status == pdPASS)
    {
      PRINT_IN_TASK("Receive success!\r\n");
      PRINT_VAR_IN_TASK(receive_data);
    }
  }
  /* USER CODE END haltCheckLedTask */
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_haltCheckLedTask */
/**
* @brief Function implementing the blinkLed1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_haltCheckLedTask */
void haltCheckLedTask(void *argument)
{
  /* USER CODE BEGIN haltCheckLedTask */
  /* Infinite loop */
  for (;;)
  {
    toggleLed4; /* Led check chip halted */
    osDelay(1000);
  }
  /* USER CODE END haltCheckLedTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
