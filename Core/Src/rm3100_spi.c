/*
 * Author: Lucas de Jesus B. Gonçalves / Antônio Emílio
 *
 * Last modified: 13/04/2023
 * Description: SPI library for the RM3100 sensor using stm32.
 */

#include "stm32l0xx_hal.h"
#include <limits.h>
#include <stdio.h>
#include <math.h>
#include <rm3100_spi.h>



uint8_t revid;
uint16_t cycle_count;
float gain = 1.0;

/* enum para facilitar na indexação dos dados */
enum {
  X2 = 0,
  X1 = 1,
  X0 = 2,
  Y2 = 3,
  Y1 = 4,
  Y0 = 5,
  Z2 = 6,
  Z1 = 7,
  Z0 = 8
};
char msg[35] = {'\0'};

/*-------------------------------FUNÇÕES INTERNAS-----------------------*/
/*Aguarda pelo pino de Data Ready*/
void wait_dr()
{
  if (USE_DR_PIN)
  {
    while (HAL_GPIO_ReadPin(GPIOB, DR_PIN) == GPIO_PIN_SET); /* checa o pino de data ready*/
    return;
  }

  uint8_t status;
  do
  {
    RM3100_SPI_READ(RM3100_REG_STATUS, &status, 1);
  } while ((status & 0x80) != 0x80); /* lê o status interno do registrador */
}

/*Formata os dados , pois não é um signed int de 24 bits*/
void data_format(RM3100_DATA *dados, uint8_t *readings)
{
  /* Manipulação de dados - não é um signed int de 24 bits */
  if (readings[X2] & 0x80)
    dados->x = 0xFF;
  if (readings[Y2] & 0x80)
    dados->y = 0xFF;
  if (readings[Z2] & 0x80)
    dados->z = 0xFF;

  dados->x = ((dados->x * 256 * 256 * 256) | (int32_t)readings[X2] * 256 * 256 | (uint16_t)readings[X1] * 256 | readings[X0])/100000;
  dados->y = ((dados->y * 256 * 256 * 256) | (int32_t)readings[Y2] * 256 * 256 | (uint16_t)readings[Y1] * 256 | readings[Y0])/100000;
  dados->z = ((dados->z * 256 * 256 * 256) | (int32_t)readings[Z2] * 256 * 256 | (uint16_t)readings[Z1] * 256 | readings[Z0])/100000;
}
/*---------------------------------------------------------------------*/

/*
 * RM3100_SPI_WRITE: Envia dados para via SPI
 * ---------------------------------------------------------------------
 * addr: valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser escrito
 * data: vetor de dados a serem enviados
 * size: numero de bytes a serem enviados
 */
void RM3100_SPI_WRITE(uint8_t addr, uint8_t *data, uint16_t size)
{

    HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET);

	/*faz manimpulação no endereço e o envia */
    uint8_t endereco = addr & 0x7F;
    HAL_SPI_Transmit(spi_handle, &endereco, 1, HAL_MAX_DELAY);

    /*
     * envia o vetor de dados - segundo a documentação da função.
     * como data já é um ponteiro, não é necessário o operador de endereço &
     */
    HAL_SPI_Transmit(spi_handle, data, size, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET);
}
/*
 * addr é o valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser lido
 */
void RM3100_SPI_READ(uint8_t addr, uint8_t *data, uint16_t size)
{
  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET);                                 // delay(100)
  uint8_t buffer[2]= {addr | 0x80, 0x00};

  HAL_SPI_Transmit(spi_handle, buffer, 2, 1000);
  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET);                    // digitalWrite(PIN_CS, HIGH)
}

//newCC is the new cycle count value (16 bits) to change the data acquisition
void changeCycleCount(){
  uint8_t CCMSB = (200 & 0xFF00) >> 8;	/* byte mais significativo */
  uint8_t CCLSB = 200 & 0xFF; 			/* byte menos significativo */

  uint8_t buffer[6] =
  {
		  CCMSB,
		  CCLSB,
		  CCMSB,
		  CCLSB,
		  CCMSB,
		  CCLSB
  };

  /* Envia o vetor de dados para ccx1. Não é necessário & 0x7F pois já está incluso na função*/
  RM3100_SPI_WRITE(RM3100_REG_CCX1, buffer, 6);
}

/* Faz a configuração da conexão com o stm32*/
void RM3100_SPI_SETUP(GPIO_InitTypeDef *GPIO_InitStruct)
{
  uint8_t revid = 0;

  RM3100_SPI_READ(RM3100_REG_REVID, &revid, 0);
  RM3100_SPI_READ(RM3100_REG_REVID, &revid, 0);
  printf("REVID ID = 0x%02X\n", revid);

  /* executa a função para mudar o cycle count*/
  changeCycleCount();

  uint32_t cycleCount = 0;
  uint8_t cc1, cc2 = 0;

  RM3100_SPI_READ(RM3100_REG_CCX1, &cc1, 0);
  RM3100_SPI_READ(RM3100_REG_CCX0, &cc2, 0);

  cycleCount = (cc1 << 8) | cc2;
  printf("Cycle Counts = %u\n", (uint) cycleCount);

 gain = (0.3671 * (float)cycleCount) + 1.5;

 uint8_t value;
  if (SINGLE_MODE)
  {
    value = 0x00;
    RM3100_SPI_WRITE(RM3100_REG_CMM, &value, 0);

    value = 0x70;
    RM3100_SPI_WRITE(RM3100_REG_POLL, &value, 0);
  }
  else
  {
    value = 0b01110101;
	RM3100_SPI_WRITE(RM3100_REG_CMM, &value, 0);
  }
}

/* Função que deve ficar em loop, retorna uma struct contendo os dados de leitura*/
RM3100_DATA RM3100_SPI_DATA()
{
  RM3100_DATA dados;

  uint8_t receive_buffer[9] = { 0 };
  uint8_t send_buffer[10] = { 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0x00 };
  // wait_dr();

  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET);

  /* automatiza o processo com um loop*/
  HAL_SPI_Transmit(spi_handle, &send_buffer[0], 1, HAL_MAX_DELAY);
  for (int i = 1; i < 10; i++)
  {
	  HAL_SPI_Transmit(spi_handle, &send_buffer[i], 1, HAL_MAX_DELAY);
	  HAL_SPI_Receive(spi_handle, &receive_buffer[i-1], 1, HAL_MAX_DELAY);
  }

  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET);

  /* Usa a função de para a formatação dos dados anteriormente implementada*/
  /*Formata os dados , pois não é um signed int de 24 bits*/
  data_format(&dados, receive_buffer);

  //calculate magnitude of results
  dados.uT = sqrt(pow(((float)(dados.x)/gain),2) + pow(((float)(dados.y)/gain),2)+ pow(((float)(dados.z)/gain),2));

  return dados;
}
