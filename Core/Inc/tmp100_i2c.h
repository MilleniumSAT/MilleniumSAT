/*
 * Author: Lucas de Jesus B. Gonçalves
 *
 * Last modified: 30/03/2013
 * Description: I2C library for the TMP100 sensor using stm32.
 */


#ifndef TMP100_I2C_H
#define TMP100_I2C_H

#include <stdint.h>
#include "stm32l0xx_hal.h"

/* Endereço i2C do sensor*/
#define TMP100_ADDR 0x49

/* Endereços dos registradores internos */
#define TEMP_REG					0x00
#define CONFIG_REG					0x01
#define TEMP_THRESH_HIGH_REG		0x02
#define TEMP_THRESH_LOW_REG			0x03

/* Shutdown Mode */
#define ENABLE_SHUTDOWN				0x01
#define DISABLE_SHUTDOWN			0x00

/* Thermostat mode */
#define COMPARATOR_MODE				0x00
#define INTERRUPT_MODE				0x02

/* Polarity */
#define POLARITY_LOW				0x00
#define POLARITY_HIGH				0x04

/* Fault queue */
#define CONSECUTIVE_FAULTS_1		0x00
#define CONSECUTIVE_FAULTS_2		0x08
#define CONSECUTIVE_FAULTS_4		0x10
#define CONSECUTIVE_FAULTS_6		0x18

/* Resolução do conversor */
#define RESOLUTION_9_BIT			0x00		//0.5 degC
#define RESOLUTION_10_BIT			0x20		//0.25 degC
#define RESOLUTION_11_BIT			0x40		//0.125 degC
#define RESOLUTION_12_BIT			0x60		//0.0625 degC

/* Constante de conversão (in degC*E-4) */
# define	MUL_9_bit				5000
# define	MUL_10_bit				2500
# define	MUL_11_bit				1250
# define	MUL_12_bit				625

typedef struct
{
	float temp;
	int status:1;
} TMP100_DATA;

/*variáveis externas (criadas no main do código do stm32)*/
extern UART_HandleTypeDef uart1;
extern I2C_HandleTypeDef hi2c1;


/*
 * addr é o valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser escrito
 */
void tmp100_write_reg(uint8_t addr, uint8_t *data);

/*
 * addr é o valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser lido
 */
uint8_t tmp100_read_reg(uint8_t addr, uint8_t *data);

/*faz a configuração do tmp100*/
void tmp100_setup();

/* Faz a configuração da conexão com o stm32*/
TMP100_DATA tmp100_loop(int conv_const);
#endif
