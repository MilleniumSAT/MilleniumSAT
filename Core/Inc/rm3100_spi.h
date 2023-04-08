/*
 * Author: Lucas de Jesus B. Gonçalves
 *
 * Last modified: 02/04/2023
 * Description: SPI library for the RM3100 sensor using stm32.
 */

#ifndef RM3100_SPI_H
#define RM3100_SPI_H

/*Definição de pino Data Ready*/
#define DR_PIN GPIO_PIN_3 //Set pin D8 to be the Data Ready Pin
#define DR_GPIO GPIOB

/*Definição do pino de CS*/
#define CS_PIN GPIO_PIN_8
#define CS_GPIO GPIOA

/* Endereços dos registradores internos sem o bit de R/W (vide datasheet) */
#define REVID_REG 0x36 	/* Endereço hexadecimal para o registrador Revid */
#define POLL_REG 0x00  	/* Endereço hexadecimal para o registrador Poll */
#define CMM_REG 0x01	/* Endereço hexadecimal para o registrador CMM */
#define STATUS_REG 0x34 /* Endereço hexadecimal para o registrador Status */
#define CCX1_REG 0x04 	/* Endereço hexadecimal para o registrador Cycle Count X1 */
#define CCX0_REG 0x05	/* Endereço hexadecimal para o registrador Cycle Count X0 */

/* Opções*/
#define INITIAL_CC  200 /* configura  cycle count*/
#define SINGLE_MODE 0 	/* 0 = modo de medida continuo; 1 = single mode */
#define USE_DR_PIN 1 	/* 0 = não usa pino de DR ; 1 = usa pino de DR */
#define UART_DBG 0		/* 0 = não printa mensagem de debug via UART, 1 = emite*/


/*
 * Estruct com as leituras nos 3 eixos em counts.
 * Para obter os dados em uT, basta dividir os valores por gain
 */
typedef struct
{
	long x;
	long y;
	long z;
	float gain;
} RM3100_DATA;

/*variáveis externas (criadas no main do código do stm32)*/
extern UART_HandleTypeDef *uart_handle;
extern SPI_HandleTypeDef *spi_handle;

/*
 * RM3100_SPI_WRITE: Envia dados para via SPI
 * ---------------------------------------------------------------------
 * addr: valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser escrito
 * data: vetor de dados a serem enviados
 * size: numero de bytes a serem enviados
 */
void RM3100_SPI_WRITE(uint8_t addr, uint8_t *data, uint16_t size);

/*
 * RM3100_SPI_READ: L~e dados via SPI
 * ---------------------------------------------------------------------
 * addr: valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser escrito
 * data: endereço do buffer de dados a ser lido
 * size: numero de bytes a serem enviados
 */
void RM3100_SPI_READ(uint8_t addr, uint8_t *data, uint16_t size);

/*
 * RM3100_SPI_CHANGE_CC: Faz a mudança do Cycle Count
 * ----------------------------------------------------------------------
 * new_cc: Novo valor de Cycle Count do rm3100
 */
void RM3100_SPI_CHANGE_CC(uint16_t new_cc);
#endif
