/*
 * Author: Lucas de Jesus B. Gonçalves
 * 
 * Last modified: 30/03/2013
 * Description: I2C library for the RM3100 sensor using stm32.
 */
#include "rm3100.h"
#include <stdio.h>

uint8_t revid;
uint16_t cycle_count;
float gain = 1.0;

enum {x2 = 0, x1 = 1, x0 = 2, y2 = 3, y1 = 4, y0 = 5, z2 = 6, z1 = 7, z0 = 8};
char msg[35] = {'\0'};

void uart_print(char *msg, int dbg_enabled)
{
	if (!dbg_enabled) return;

	HAL_UART_Transmit(&huart2, (uint8_t *) msg, sizeof(msg), 100);
	HAL_Delay(1000);
}

/*
 * addr é o valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser escrito
 */
void rm3100_write_reg(uint8_t addr, uint8_t *data){
	/* << 1 por conta do endereçamento de  7 bits (o ultimo bit é definido pelo i2c)*/
	HAL_I2C_Mem_Write(&hi2c1, (RM3100_ADDR << 1), (addr << 1), I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );
}

/*
 * addr é o valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser lido
 */
void rm3100_read_reg(uint8_t addr, uint8_t *data)
{
  /* << 1 por conta do endereçamento de  7 bits (o ultimo bit é definido pelo i2c)*/
  HAL_I2C_Mem_Read(&hi2c1, (RM3100_ADDR << 1), (addr << 1), I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );
}


/* Faz a mudança do Cycle Count*/
void change_cc(uint16_t new_cc)
{
	uint8_t CCMSB = (new_cc & 0xFF00) >> 8; /* pega o byte mais significativo */
	uint8_t CCLSB = new_cc & 0xFF; 			/* pega o byte menos significativo */

	uint8_t buffer[]={
			CCMSB,  /* write new cycle count to ccx1 */
			CCLSB,  /* write new cycle count to ccx0 */
			CCMSB,  /* write new cycle count to ccy1 */
			CCLSB,  /* write new cycle count to ccy0 */
			CCMSB,  /* write new cycle count to ccz1 */
			CCLSB  	/* write new cycle count to ccz0 */
	};

	/* << 1 por conta do endereçamento de  7 bits (o ultimo bit é definido pelo i2c)*/
	HAL_I2C_Mem_Write(&hi2c1, (RM3100_ADDR << 1), (CCX1_REG << 1), I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY );
}

/* Faz a configuração da conexão com o stm32*/
void rm3100_setup(GPIO_InitTypeDef *GPIO_InitStruct)
{
	/*Definição de pino Data Ready*/
	GPIO_InitStruct->Pin = DR_PIN;
	GPIO_InitStruct->Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct->Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, GPIO_InitStruct);

	/*faz a leitura do Revid e em seguida printa no uart*/
	rm3100_read_reg(REVID_REG, &revid);

	/*deve ser 0x22*/
	sprintf(msg, "REVID ID = 0x%d", revid);
	uart_print(msg, UART_DBG);

	/*Altera o cycle count*/
	change_cc((uint16_t) INITIAL_CC);

	uint8_t cc_tmp1, cc_tmp2;
	rm3100_read_reg(CCX1_REG, &cc_tmp1);
	rm3100_read_reg(CCX0_REG, &cc_tmp2);

	cycle_count = (((uint16_t) cc_tmp1) << 8) | ((uint16_t) cc_tmp2);

	sprintf(msg, "Cycles count = %d", cycle_count);
	uart_print(msg, UART_DBG);

	/* Equação linear para calcular o ganho por meio de cycle_count */
	gain = (0.3671 * (float)cycle_count) + 1.5;

	sprintf(msg, "Gain = %u.%u", (unsigned int) gain*100 / 100, (unsigned int) gain*100%100);
	uart_print(msg, UART_DBG);

	uint8_t tmp; /*rm3100_write_reg exige um ponteiro*/
	if (SINGLE_MODE)
	{	/*Ativa o single mode*/
		tmp  = 0;
		rm3100_write_reg(CMM_REG, &tmp);
		tmp  = 0x70;
		rm3100_write_reg(POLL_REG, &tmp);
	}
	else
	{
		/*Permite a transmissão de medidas contínuas com as funções de alarme desligadas */
		tmp = 0x79;
		rm3100_write_reg(CMM_REG, &tmp);
	}
}


void wait_dr()
{
	if(USE_DR_PIN)
	{
		while(HAL_GPIO_ReadPin(GPIOE, DR_PIN) == GPIO_PIN_RESET); /* checa o pino de data ready*/
		return;
	}
	uint8_t status;
	do
	{
		rm3100_read_reg(STATUS_REG, &status);
	}
	while((status & 0x80) != 0x80); /* lê o status interno do registrador */
}

void data_format(RM3100_DATA *dados, uint8_t *readings)
{
	/* Manipulação de dados - não é um signed int de 24 bits */
	if (readings[x2] & 0x80) dados->x = 0xFF;
	if (readings[y2] & 0x80) dados->y = 0xFF;
	if (readings[z2] & 0x80) dados->z = 0xFF;


	dados->x = (dados->x*256*256*256)|(int32_t) readings[x2]*256*256|(uint16_t) readings[x1]*256|readings[x0];
	dados->y = (dados->y*256*256*256)|(int32_t) readings[y2]*256*256|(uint16_t) readings[y1]*256|readings[y0];
	dados->z = (dados->z*256*256*256)|(int32_t) readings[z2]*256*256|(uint16_t) readings[z1]*256|readings[z0];
}

/* Função que deve ficar em loop, retorna uma struct contendo os dados de leitura*/
RM3100_DATA rm3100_loop()
{
	RM3100_DATA dados;
	dados.x = 0;
	dados.y = 0;
	dados.z = 0;
	dados.gain = gain;

	uint8_t readings[9]; /* valores de leitura : x2,x1,x0,y2,y1,y0,z2,z1,z0 */
	uint8_t tmp;

	/* Aguarda até que um dos 2 métodos esteja pronto para receber os dados*/
	wait_dr();

	/* requisita do primeiro registrador de resultados */
	tmp = 0x24;
	HAL_I2C_Master_Transmit(&hi2c1, (RM3100_ADDR << 1), &tmp, 1, HAL_MAX_DELAY);

	/* faz a leitura da requisição */
	HAL_I2C_Master_Receive(&hi2c1, (RM3100_ADDR << 1), readings, 9, HAL_MAX_DELAY);

	/* formata os resultados em valores de 32 bits (com sinal) */
	data_format(&dados, readings);

	return dados;
}
