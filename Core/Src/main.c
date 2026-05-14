/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>

#include "arith_encoder.h"
#include "config.h"
#include "uart_protocol.h"

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef enum {
  APP_STATE_IDLE = 0,
  APP_STATE_RECEIVING_INPUT
} app_state_t;

static uart_protocol_t g_protocol;
static arith_ctx_t g_arith;
static app_state_t g_state = APP_STATE_IDLE;

static uint32_t g_expected_input_size = 0u;
static uint32_t g_received_input_size = 0u;
static uint32_t g_total_compressed_size = 0u;
static bool g_encoder_started = false;

static uint8_t g_output_staging[APP_UART_TX_PAYLOAD_MAX];
static uint16_t g_output_staging_len = 0u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static bool app_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len);
static void app_send_ack(uint8_t cmd);
static void app_send_nack(uint8_t error_code);
static void app_send_error(uint8_t error_code);
static void app_reset_session(void);
static bool app_flush_output_staging(void);
static arith_status_t app_output_emit(uint8_t byte, void *user);
static void app_handle_frame(const uart_frame_t *frame);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool app_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
  uint8_t tx_buffer[APP_UART_FRAME_OVERHEAD + APP_UART_TX_PAYLOAD_MAX];
  uint16_t frame_size = uart_protocol_build_frame(cmd, payload, len, tx_buffer, (uint16_t)sizeof(tx_buffer));

  if (frame_size == 0u) {
    return false;
  }

  return HAL_UART_Transmit(&huart2, tx_buffer, frame_size, APP_UART_TX_TIMEOUT_MS) == HAL_OK;
}

static void app_send_ack(uint8_t cmd) {
  uint8_t payload[1];
  payload[0] = cmd;
  (void)app_send_frame(CMD_ACK, payload, 1u);
}

static void app_send_nack(uint8_t error_code) {
  uint8_t payload[1];
  payload[0] = error_code;
  (void)app_send_frame(CMD_NACK, payload, 1u);
}

static void app_send_error(uint8_t error_code) {
  uint8_t payload[1];
  payload[0] = error_code;
  (void)app_send_frame(CMD_ERROR, payload, 1u);
}

static void app_reset_session(void) {
  g_state = APP_STATE_IDLE;
  g_expected_input_size = 0u;
  g_received_input_size = 0u;
  g_total_compressed_size = 0u;
  g_encoder_started = false;
  g_output_staging_len = 0u;
  arith_init(&g_arith);
}

static bool app_flush_output_staging(void) {
  if (g_output_staging_len == 0u) {
    return true;
  }
  if (!app_send_frame(CMD_COMPRESSED_DATA, g_output_staging, g_output_staging_len)) {
    return false;
  }
  g_output_staging_len = 0u;
  return true;
}

static arith_status_t app_output_emit(uint8_t byte, void *user) {
  (void)user;
  g_output_staging[g_output_staging_len] = byte;
  g_output_staging_len++;
  g_total_compressed_size++;

  if (g_output_staging_len >= APP_UART_TX_PAYLOAD_MAX) {
    if (!app_flush_output_staging()) {
      return ARITH_STATUS_OUTPUT_ERROR;
    }
  }

  return ARITH_STATUS_OK;
}

static void app_handle_frame(const uart_frame_t *frame) {
  uint16_t i;

  if (frame == 0) {
    return;
  }

  switch (frame->cmd) {
  case CMD_START:
    if (frame->len != 4u) {
      app_send_nack(ERR_BAD_STATE);
      return;
    }

    g_expected_input_size = (uint32_t)frame->payload[0]
        | ((uint32_t)frame->payload[1] << 8u)
        | ((uint32_t)frame->payload[2] << 16u)
        | ((uint32_t)frame->payload[3] << 24u);

    if (g_expected_input_size > APP_MAX_INPUT_BYTES) {
      app_send_nack(ERR_OVERSIZE);
      app_reset_session();
      return;
    }

    app_reset_session();
    g_expected_input_size = (uint32_t)frame->payload[0]
        | ((uint32_t)frame->payload[1] << 8u)
        | ((uint32_t)frame->payload[2] << 16u)
        | ((uint32_t)frame->payload[3] << 24u);
    if (arith_start(&g_arith, app_output_emit, 0) != ARITH_STATUS_OK) {
      app_send_error(ERR_ENCODER);
      app_reset_session();
      return;
    }
    g_encoder_started = true;
    g_state = APP_STATE_RECEIVING_INPUT;
    app_send_ack(CMD_START);
    break;

  case CMD_DATA:
    if (g_state != APP_STATE_RECEIVING_INPUT) {
      app_send_nack(ERR_BAD_STATE);
      return;
    }
    if (frame->len == 0u) {
      app_send_nack(ERR_BAD_STATE);
      return;
    }
    if ((g_received_input_size + frame->len) > g_expected_input_size) {
      app_send_nack(ERR_OVERSIZE);
      app_reset_session();
      return;
    }

    for (i = 0u; i < frame->len; i++) {
      if (!g_encoder_started) {
        app_send_error(ERR_ENCODER);
        app_reset_session();
        return;
      }
      if (arith_process_byte(&g_arith, frame->payload[i]) != ARITH_STATUS_OK) {
        app_send_error(ERR_ENCODER);
        app_reset_session();
        return;
      }
      g_received_input_size++;
    }

    app_send_ack(CMD_DATA);
    break;

  case CMD_END_INPUT:
  case CMD_RUN:
    if (g_state != APP_STATE_RECEIVING_INPUT) {
      app_send_nack(ERR_BAD_STATE);
      return;
    }

    if (g_received_input_size != g_expected_input_size) {
      app_send_nack(ERR_BAD_STATE);
      app_reset_session();
      return;
    }

    if (!g_encoder_started || (arith_finish(&g_arith) != ARITH_STATUS_OK)) {
      app_send_error(ERR_ENCODER);
      app_reset_session();
      return;
    }

    if (!app_flush_output_staging()) {
      app_send_error(ERR_ENCODER);
      app_reset_session();
      return;
    }

    {
      uint8_t payload[4];
      payload[0] = (uint8_t)(g_total_compressed_size & 0xFFu);
      payload[1] = (uint8_t)((g_total_compressed_size >> 8u) & 0xFFu);
      payload[2] = (uint8_t)((g_total_compressed_size >> 16u) & 0xFFu);
      payload[3] = (uint8_t)((g_total_compressed_size >> 24u) & 0xFFu);
      (void)app_send_frame(CMD_END_OUTPUT, payload, 4u);
    }

    app_reset_session();
    break;

  default:
    app_send_nack(ERR_BAD_STATE);
    break;
  }
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uart_protocol_init(&g_protocol);
  arith_init(&g_arith);
  app_reset_session();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t rx_byte = 0u;
    uart_frame_t frame;

    if (HAL_UART_Receive(&huart2, &rx_byte, 1u, APP_UART_RX_TIMEOUT_MS) == HAL_OK) {
      if (uart_protocol_process_byte(&g_protocol, rx_byte, &frame)) {
        app_handle_frame(&frame);
      } else if (g_protocol.error_code != 0u) {
        app_send_nack(g_protocol.error_code);
        g_protocol.error_code = 0u;
      }
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
