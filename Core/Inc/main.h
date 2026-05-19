/* USER CODE BEGIN Header */
/**
 * @file      main.h
 * @brief     Cabeçalho de Definições Globais e Protótipos de Periféricos do Sistema.
 * @details   Apresentação Geral: Arquivo gerador e integrador do ecossistema HAL do STM32Cube. Declara manipuladores de exceções globais.
 * Permissões de Uso: Copyright (c) 2026 STMicroelectronics. Todos os direitos reservados. Uso estrito sob os termos de licença de software embarcado ST.
 * Como usar: Incluído obrigatoriamente por todos os arquivos de origem C que interagem com registradores de hardware e APIs HAL.
 * Entrada: N/A.
 * Saída: N/A.
 * Contexto: Infraestrutura base para o trabalho da disciplina de SEMB.
 * Plataforma Alvo: STM32F030R8T6.
 *
 * @author    STMicroelectronics, Paulo Vinícius Holanda Gomes, Pedro Lucas Coutinho de Araujo (Extensões)
 * @date      Maio de 2026
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Esta função é executada em caso de ocorrência de erro fatal nos periféricos HAL.
  * @details Desabilita interrupções globais e trava a CPU em um loop infinito preventivo.
  * @retval None
  */
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
