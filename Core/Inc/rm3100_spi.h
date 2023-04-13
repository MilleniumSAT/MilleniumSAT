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

/* Endereços dos registradores internos */
#define	RM3100_REG_POLL		0x00	/* Endereço hexadecimal do registrador Poll*/
#define RM3100_REG_CMM  	0x01	/* Endereço hexadecimal do registrador do modo de medição contínua*/
#define RM3100_REG_CCX  	0x04 	/* Cycle counts -- X axis */
#define RM3100_REG_CCY  	0x06 	/* cycle counts -- Y axis */
#define RM3100_REG_CCZ  	0x08 	/* cycle counts -- Z axis */
#define RM3100_REG_TMRC 	0x0B 	/* Ativa o modo de dados contínuos */
#define RM3100_REG_MX   	0x24 	/* Medidas -- eixo X */
#define RM3100_REG_MY   	0x27 	/* Medidas -- eixo Y */
#define RM3100_REG_MZ   	0x2A	/* Medidas -- eixo Z */
#define RM3100_REG_BIST 	0x33	/* Autoteste interno */
#define RM3100_REG_STATUS	0x34	/* Endereço hexadecimal do registrador de status DRDY */
#define RM3100_REG_HSHAKE  	0x35 	/* Endereço de handshake*/
#define RM3100_REG_REVID 	0x36 	/* MagI2C revision identification */
#define RM3100_REG_CCX1 	0x04   	/* Endereço hexadecimal do registrador Cycle Count X1 */
#define RM3100_REG_CCX0 	0x05   	/* Endereço hexadecimal do registrador Cycle Count X0 */

#define SMM_AXIS_X	0x10	/* (1 << 4) */
#define SMM_AXIS_Y  0x20    /* (1 << 5) */
#define SMM_AXIS_Z  0x40    /* (1 << 6) */

#define CMM_AXIS_X	0x10	/* (1 << 4) */
#define CMM_AXIS_Y  0x20    /* (1 << 5) */
#define CMM_AXIS_Z  0x40    /* (1 << 6) */


/* Opções*/
#define INITIAL_CC  200 /* configura  cycle count*/
#define CYCLE_COUNTS_DEFAULT        (200)

#define SINGLE_MODE 0 	/* 0 = modo de medida continuo; 1 = single mode */
#define USE_DR_PIN 1 	/* 0 = não usa pino de DR ; 1 = usa pino de DR */
#define UART_DBG 0		/* 0 = não printa mensagem de debug via UART, 1 = emite*/

/* CMMode config */
#define CMM_DRDM_ALL_AXIS	0x00	/* (0 << 2) */
#define CMM_DRDM_ANY_AXIS	0x04    /* (1 << 2) */

/* Comunicação SPI*/
#define SPI_MAX_SEND 32




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
//extern UART_HandleTypeDef *uart_handle;
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

/*
 * RM3100_SPI_SETUP: Faz a configuração da conexão com o stm32
 * ----------------------------------------------------------------------
 */
void RM3100_SPI_SETUP(GPIO_InitTypeDef *GPIO_InitStruct);

/*
 *  RM3100_SPI_DATA: Função que deve ficar em loop, retorna uma struct contendo os dados de leitura
 *  ---------------------------------------------------------------------
 */
RM3100_DATA RM3100_SPI_DATA();
#endif
