/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Loop principal e Máquina de Estados da Aplicação (FSM).
  * @details        Apresentação Geral: Orquestra a recepção UART e repassa os bytes para o motor aritmético.
  * Permissões de Uso: Código de uso acadêmico (SEMB). Copyright (c) 2026 STMicroelectronics para código gerado, lógica de compressão livre.
  * Como usar: O firmware deve ser gravado na STM32. A placa aguarda frames UART definidos pelo protocolo.
  * Entrada: Fluxo de bytes seriais estruturados (SOF, CMD, LEN, PAYLOAD, CHK).
  * Saída: Frames UART de ACK ou blocos de bytes comprimidos (COMPRESSED_DATA).
  * Contexto: Trabalho da disciplina de Sistemas Embarcados (SEMB).
  * Plataforma Alvo: STM32F030R8T6
  *
  * @author         Paulo Vinícius Holanda Gomes, Pedro Lucas Coutinho de Araujo
  * @date           Maio de 2026
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

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/**
 * @brief Enumeração dos estados do fluxo de compressão na aplicação.
 */
typedef enum {
  APP_STATE_IDLE = 0,         /**< Sistema ocioso aguardando CMD_START */
  APP_STATE_RECEIVING_INPUT   /**< Sistema processando recepção de CMD_DATA */
} app_state_t;

static uart_protocol_t g_protocol;                          /**< Contexto do parser UART */
static arith_ctx_t g_arith;                                 /**< Contexto matemático do codificador aritmético */
static app_state_t g_state = APP_STATE_IDLE;                /**< Estado atual da aplicação */

static uint32_t g_expected_input_size = 0u;                 /**< Tamanho declarado no pacote START */
static uint32_t g_received_input_size = 0u;                 /**< Quantidade de bytes efetivamente processados */
static uint32_t g_total_compressed_size = 0u;               /**< Quantidade de bytes gerados pelo codificador */
static bool g_encoder_started = false;                      /**< Flag de ativação segura do encoder */

static uint8_t g_output_staging[APP_UART_TX_PAYLOAD_MAX];   /**< Buffer de acumulação de saída comprimida */
static uint16_t g_output_staging_len = 0u;                  /**< Cursor de preenchimento do buffer de saída */

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
/**
 * @brief  Monta e transmite um frame UART bloqueante via HAL.
 * @param  cmd Identificador do comando (ex: CMD_ACK, CMD_COMPRESSED_DATA).
 * @param  payload Ponteiro para os dados a serem enviados (pode ser NULL se len=0).
 * @param  len Tamanho em bytes do payload.
 * @return bool Retorna true se a transmissão HAL obteve sucesso, false em timeout ou erro.
 * @note   Globais afetadas: Nenhuma. Interage fisicamente com huart2.
 */
static bool app_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
  uint8_t tx_buffer[APP_UART_FRAME_OVERHEAD + APP_UART_TX_PAYLOAD_MAX];
  uint16_t frame_size = uart_protocol_build_frame(cmd, payload, len, tx_buffer, (uint16_t)sizeof(tx_buffer));

  if (frame_size == 0u) {
    return false;
  }

  return HAL_UART_Transmit(&huart2, tx_buffer, frame_size, APP_UART_TX_TIMEOUT_MS) == HAL_OK;
}

/**
 * @brief  Emite frame de confirmação positiva (ACK).
 * @param  cmd Código do comando original que está sendo confirmado.
 * @return void
 */
static void app_send_ack(uint8_t cmd) {
  uint8_t payload[1];
  payload[0] = cmd;
  (void)app_send_frame(CMD_ACK, payload, 1u);
}

/**
 * @brief  Emite frame de rejeição (NACK) para erros de fluxo.
 * @param  error_code Código de erro para o Host.
 * @return void
 */
static void app_send_nack(uint8_t error_code) {
  uint8_t payload[1];
  payload[0] = error_code;
  (void)app_send_frame(CMD_NACK, payload, 1u);
}

/**
 * @brief  Emite frame de erro fatal do motor de compressão.
 * @param  error_code Código de erro do encoder.
 * @return void
 */
static void app_send_error(uint8_t error_code) {
  uint8_t payload[1];
  payload[0] = error_code;
  (void)app_send_frame(CMD_ERROR, payload, 1u);
}

/**
 * @brief  Zera variáveis da sessão, purga contextos e retorna estado para IDLE.
 * @return void
 * @note   Globais afetadas: g_state, g_expected_input_size, g_received_input_size, g_total_compressed_size, g_encoder_started, g_output_staging_len, g_arith.
 */
static void app_reset_session(void) {
  g_state = APP_STATE_IDLE;
  g_expected_input_size = 0u;
  g_received_input_size = 0u;
  g_total_compressed_size = 0u;
  g_encoder_started = false;
  g_output_staging_len = 0u;
  arith_init(&g_arith);
}

/**
 * @brief  Transmite via UART os dados acumulados no buffer de saída comprimida e o esvazia.
 * @return bool True se enviado com sucesso ou se vazio; False em caso de falha de envio.
 * @note   Globais afetadas: g_output_staging_len é zerado após envio.
 */
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

/**
 * @brief  Callback executado pelo codificador aritmético sempre que 8 bits (1 byte) comprimidos são formados.
 * @param  byte O byte bruto comprimido.
 * @param  user Ponteiro opcional de contexto (não utilizado).
 * @return arith_status_t Status indicando sucesso ou falha na emissão UART prematura (early compressed).
 * @note   Globais afetadas: g_output_staging, g_output_staging_len, g_total_compressed_size. Invoca app_flush_output_staging() ao lotar o buffer.
 */
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

/**
 * @brief  Processa as requisições (frames lógicos) recebidos pelo parser serial.
 * @details Executa transições de estado (IDLE -> RECEIVING), consome CMD_DATA injetando no encoder e trata conclusão (CMD_END_INPUT).
 * @param  frame Ponteiro constante para estrutura uart_frame_t completamente validada pelo checksum.
 * @return void
 * @note   Globais afetadas: Toda a árvore de variáveis globais estáticas do módulo, dependendo do comando.
 */
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
  * @brief  The application entry point. Ponto de entrada principal Baremetal.
  * @details Inicializa HAL, clock, GPIO, UART e executa o loop de recepção UART + despacho de frames para a FSM da aplicação.
  * @note   Globais afetadas: `g_protocol`, `g_arith` e toda a sessão controlada por `app_handle_frame`.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  uart_protocol_init(&g_protocol);
  arith_init(&g_arith);
  app_reset_session();

  while (1)
  {
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
}

/**
 * @brief  Configura a árvore de clocks do sistema para operar com HSI interno sem PLL.
 * @details Define SYSCLK, HCLK e PCLK1, aplicando latência de flash compatível.
 * @return void
 * @note   Globais afetadas: Nenhuma variável global de aplicação; afeta registradores RCC/FLASH do microcontrolador.
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
 * @brief  Inicializa a USART2 no modo assíncrono padrão 8N1.
 * @details Configura baudrate, formato de quadro, direção TX/RX e chama `HAL_UART_Init`.
 * @return void
 * @note   Globais afetadas: Atualiza a estrutura global `huart2`.
 */
static void MX_USART2_UART_Init(void)
{
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
}

/**
 * @brief  Habilita o clock do banco GPIO utilizado pela aplicação.
 * @return void
 * @note   Globais afetadas: Nenhuma variável global de aplicação; afeta registradores RCC de GPIO.
 */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
 * @brief  Tratador de erro fatal da aplicação.
 * @details Desabilita interrupções globais e mantém a CPU em loop infinito para evitar comportamento indefinido.
 * @return void
 * @note   Globais afetadas: Nenhuma variável global de aplicação.
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
