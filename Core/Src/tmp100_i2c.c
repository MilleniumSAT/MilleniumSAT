/*
 * Author: Lucas de Jesus B. Gonçalves
 *
 * Last modified: 02/04/2023
 * Description: I2C library for the TMP3100 sensor using stm32.
 */
#include "stm32l0xx_hal.h"
#include <limits.h>
#include <tmp100_i2c.h>

HAL_StatusTypeDef tmp100_status;

/*
 * addr é o valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser escrito
 */
void tmp100_write_reg(uint8_t addr, uint8_t *data){
	/* << 1 por conta do endereçamento de  7 bits (o ultimo bit é definido pelo i2c)*/
	HAL_I2C_Mem_Write(&hi2c1, (TMP100_ADDR << 1), (addr << 1), I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );
}

/*
 * addr é o valor de 7 bits do endereço do registrador, data é o ponteiro para um buffer de 16 bits
 * referente ao dado a ser lido
 */
uint8_t tmp100_read_reg(uint8_t addr, uint8_t *data)
{
	/* << 1 por conta do endereçamento de  7 bits (o ultimo bit é definido pelo i2c)*/
	tmp100_status = HAL_I2C_Mem_Read(&hi2c1, (TMP100_ADDR << 1), (addr << 1), I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY );

	if ( tmp100_status != HAL_OK )
		return 1;
	return 0;
}

/*faz a configuração do tmp100*/
void tmp100_setup()
{
	/* modo parão: disable shutdown, comparator mode, active low, 1 consecutive fault
	 * resolução de 12 bits
	 */
	uint8_t resolution = RESOLUTION_12_BIT;
	tmp100_write_reg(CONFIG_REG, &resolution);
}
TMP100_DATA tmp100_loop(int conv_const)
{
	TMP100_DATA dados;

	/* Faz a leitura do ADC*/
	uint8_t adc_data[2]={0,0};
	dados.status = tmp100_read_reg(TEMP_REG, adc_data);

	/* combina os bytes */
	int16_t val = ((int16_t)adc_data[0] << 4) | (adc_data[1] >> 4);

	/* Converte para complemento de 2, pq a temperatura pode ser negativa */
	if ( val > 0x7FF )
	{
		val |= 0xF000;
	}

	/*converte para celcius (e se der problema na leitura retorna SHRT_MAX) */
	dados.temp = dados.status? SHRT_MAX : (float) val * conv_const/10000;
	return dados;
}
