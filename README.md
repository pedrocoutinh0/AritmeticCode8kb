# Projeto de Compressão Aritmética Embarcada

**Autores:** Paulo Vinícius Holanda Gomes, Pedro Lucas Coutinho de Araujo  
**Data:** Maio de 2026  
**Contexto:** Trabalho da disciplina de Sistemas Embarcados (SEMB)  
**Plataforma Alvo:** STM32F030R8T6 

## Visão Geral
Este projeto implementa um codificador aritmético adaptativo em hardware de recursos limitados (STM32). O processamento ocorre em modelo de *streaming* (byte a byte) para contornar a restrição de RAM da placa, utilizando comunicação UART bidirecional com controle de fluxo *Stop-and-Wait*.

## Hierarquia de Arquivos e Responsabilidades

* `Core/Inc/config.h`
  * **Função:** Centraliza as macros de configuração global do projeto. Define limites operacionais de memória, timeouts de hardware, parâmetros da matemática do codificador e os códigos hexadecimais dos comandos da camada de aplicação UART.
* `Core/Inc/main.h`
  * **Função:** Definições globais e protótipos de periféricos do sistema gerados pelo STM32Cube.
* `Core/Inc/arith_encoder.h`
  * **Função:** Definições de tipo, estruturas de dados (como `arith_ctx_t`) e protótipos para o motor aritmético.
* `Core/Inc/uart_protocol.h`
  * **Função:** Estruturas de dados (`uart_frame_t`, `uart_protocol_t`) e protótipos para a FSM do enlace serial.
* `Core/Src/main.c`
  * **Função:** Ponto de entrada da aplicação (*entry point*). Implementa a Máquina de Estados da Aplicação (FSM). Interliga a recepção UART ao algoritmo de compressão e gerencia os buffers de transmissão de volta ao *host*.
* `Core/Src/arith_encoder.c`
  * **Função:** Motor do algoritmo de compressão aritmética. Contém o modelo estatístico (tabela de frequências dinâmicas) e a lógica de subdivisão de limites (precisão de 16-bits) com renormalização para evitar *underflow/overflow*.
* `Core/Src/uart_protocol.c`
  * **Função:** Camada de enlace e *Parser* da UART. Processa o fluxo serial assíncrono em busca de delimitadores (SOF), valida pacotes utilizando *checksum* (XOR) e isola o *payload* útil em *frames* lógicos para a aplicação.
* `python/send_file_uart.py`
  * **Função:** Orquestrador no lado do *Host* (PC). Fatia o arquivo original em blocos (chunks), envia para a MCU controlando o fluxo via `ACK`, acumula o *bitstream* recebido precocemente e encapsula o resultado no formato contêiner `ACS1`.
* `python/restore_file.py`
  * **Função:** Validador e descompressor *Host-side*. Extrai os metadados do contêiner `ACS1` e executa a operação inversa do codificador, restaurando o arquivo original e validando a integridade *byte a byte*.

## Entrada e Saída da Aplicação
- **Entrada (MCU):** Frames UART no formato `SOF + CMD + LEN + PAYLOAD + CHECKSUM` com bytes de arquivo bruto.
- **Saída (MCU):** Frames de controle (`ACK`, `NACK`, `ERROR`) e dados comprimidos (`CMD_COMPRESSED_DATA` + `CMD_END_OUTPUT`).
- **Tipo de dado de entrada:** fluxo binário (`uint8_t`) enviado em blocos.
- **Tipo de dado de saída:** fluxo binário comprimido (`uint8_t`) com metadados de tamanho final.

## Como Executar (Help Rápido)
1. Compilar e gravar o firmware da pasta `Core/` na placa STM32F030R8T6.
2. Conectar a UART da placa ao host e ajustar baudrate em `115200`.
3. Executar no host o envio do arquivo por `python/send_file_uart.py`.
4. Validar a restauração por `python/restore_file.py`.

## Procedimentos de Teste e Validação
- **Teste funcional básico:** enviar arquivo pequeno conhecido (ex.: texto curto), comprimir e restaurar, comparando byte a byte o original e o restaurado.
- **Teste de integridade de enlace:** injetar quadro com checksum inválido e verificar retorno `CMD_NACK` com `ERR_BAD_CHECKSUM`.
- **Teste de máquina de estados:** enviar `CMD_DATA` sem `CMD_START` e validar rejeição com `ERR_BAD_STATE`.
- **Teste de limite de memória:** declarar tamanho maior que `APP_MAX_INPUT_BYTES` e confirmar `ERR_OVERSIZE`.
- **Teste de fim de fluxo:** após `CMD_END_INPUT`, conferir emissão de `CMD_END_OUTPUT` com tamanho comprimido consistente.

## Ajustes para Migração de Plataforma
- **UART e clock:** revisar inicialização em `Core/Src/main.c` (`MX_USART2_UART_Init` e `SystemClock_Config`) conforme o novo MCU.
- **Pinos físicos:** atualizar mapeamento de GPIO/UART no STM32CubeMX e regenerar arquivos HAL do projeto.
- **Limites de memória:** ajustar macros em `Core/Inc/config.h` (`APP_MAX_INPUT_BYTES`, tamanhos de payload e timeouts).
- **Desempenho/robustez:** recalibrar `APP_UART_RX_TIMEOUT_MS` e `APP_UART_TX_TIMEOUT_MS` conforme a nova frequência de clock e qualidade de enlace.
- **Compatibilidade do protocolo:** manter inalterados `CMD_*`, estrutura de frame e checksum para interoperar com scripts host existentes.