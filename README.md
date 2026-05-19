# Projeto de Compressão Aritmética Embarcada

**Autores:** Paulo Vinícius Holanda Gomes, Pedro Lucas Coutinho de Araujo  
**Data:** Maio de 2026  
**Contexto:** Trabalho da disciplina de Sistemas Embarcados (SEMB)  
**Plataforma Alvo:** STM32F030R8T6 / STM32F030F4P6  

## Visão Geral
Este projeto implementa um codificador aritmético adaptativo em hardware de recursos limitados (STM32). O processamento ocorre em modelo de *streaming* (byte a byte) para contornar a restrição de RAM da placa, utilizando comunicação UART bidirecional com controle de fluxo *Stop-and-Wait*.

## Hierarquia de Arquivos e Responsabilidades

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
  * **Função:** Orquestrador no lado do *Host* (PC). Fatia o arquivo original em blocos (chunks), envia para a MCU controlando o fluxo via `ACK`, acumular o *bitstream* recebido precocemente e encapsular o resultado no formato contêiner `ACS1`.
* `python/restore_file.py`
  * **Função:** Validador e descompressor *Host-side*. Extrai os metadados do contêiner `ACS1` e executa a operação inversa do codificador, restaurando o arquivo original e validando a integridade *byte a byte*.
