/*
 * Author: Lucas de Jesus B. Gonçalves
 *
 * Last modified: 30/03/2013
 * Description: I2C library for the RM3100 sensor using stm32.
 */

#ifndef RM3100_H
#define RM3100_H

#include <stdint.h>
#include "stm32l0xx_hal.h"


/* Endereço hexadecimal escravo para o RM3100.
 * 5 primeiros bits são marcados como 0b01000 e os outros 2 bits pelos pinos 3 e 28.
 * Assim para 3 High e 28 Low, temos 0b0100001 = 0x21
 */
#define RM3100_ADDR 0x21

/*Definição de pino Data Ready*/
#define DR_PIN GPIO_PIN_8 //Set pin D8 to be the Data Ready Pin

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
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

/*
 * addr é o valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser escrito
 */
void rm3100_write_reg(uint8_t addr, uint8_t *data);

/*
 * addr é o valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser lido
 */
void rm3100_read_reg(uint8_t addr, uint8_t *data);

/*
 *  Faz a mudança do Cycle Count. Default = 200
 * (Cycle count mais baixo -> alta taxa de dados, mas porém com menor resolução)
 */
void change_cc(uint16_t new_cc);

/* Faz a configuração da conexão com o stm32*/
void rm3100_setup(GPIO_InitTypeDef *GPIO_InitStruct);

/* Função que deve ficar em loop, retorna uma struct contendo os dados de leitura*/
RM3100_DATA rm3100_loop();

/* printa mensagem via uart*/
void uart_print(char *msg, int dbg_enabled);
#endif




