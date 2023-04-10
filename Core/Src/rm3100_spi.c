/*
 * Author: Lucas de Jesus B. Gonçalves
 *
 * Last modified: 02/04/2023
 * Description: SPI library for the RM3100 sensor using stm32.
 */

#include "stm32l0xx_hal.h"
#include <limits.h>
#include <stdio.h>
#include <rm3100_spi.h>

#define SPI_MAX_SEND 32
#define RM3100_REVID_REG 0x36  // Hexadecimal address for the Revid internal register
#define RM3100_POLL_REG 0x00   // Hexadecimal address for the Poll internal register
#define RM3100_CMM_REG 0x01    // Hexadecimal address for the Continuous Measurement Mode internal register
#define RM3100_STATUS_REG 0x34 // Hexadecimal address for the Status internal register
#define RM3100_CCX1_REG 0x04   // Hexadecimal address for the Cycle Count X1 internal register
#define RM3100_CCX0_REG 0x05   // Hexadecimal address for the Cycle Count X0 internal register

uint8_t revid;
uint16_t cycle_count;
float gain = 1.0;

enum
{
  x2 = 0,
  x1 = 1,
  x0 = 2,
  y2 = 3,
  y1 = 4,
  y0 = 5,
  z2 = 6,
  z1 = 7,
  z0 = 8
};
char msg[35] = {'\0'};

/*-------------------------------FUNÇÕES INTERNAS-----------------------*/

/*Exibe mensagem via uart*/
void uart_print(char *msg, int dbg_enabled)
{
  //	if (!dbg_enabled) return;
  //
  //	HAL_UART_Transmit(uart_handle, (uint8_t *) msg, sizeof(msg), 100);
  //	HAL_Delay(1000);
}

/*Aguarda pelo pino de Data Ready*/
void wait_dr()
{
  if (USE_DR_PIN)
  {
    while (HAL_GPIO_ReadPin(GPIOB, DR_PIN) == GPIO_PIN_SET)
      ; /* checa o pino de data ready*/
    return;
  }
  else
  {
    return;
  }

  uint8_t status;
  do
  {
    RM3100_SPI_READ(STATUS_REG, &status, 1);
  } while ((status & 0x80) != 0x80); /* lê o status interno do registrador */
}

/*Formata os dados , pois não é um signed int de 24 bits*/
void data_format(RM3100_DATA *dados, uint8_t *readings)
{
  /* Manipulação de dados - não é um signed int de 24 bits */
  if (readings[x2] & 0x80)
    dados->x = 0xFF;
  if (readings[y2] & 0x80)
    dados->y = 0xFF;
  if (readings[z2] & 0x80)
    dados->z = 0xFF;

  dados->x = (dados->x * 256 * 256 * 256) | (int32_t)readings[x2] * 256 * 256 | (uint16_t)readings[x1] * 256 | readings[x0];
  dados->y = (dados->y * 256 * 256 * 256) | (int32_t)readings[y2] * 256 * 256 | (uint16_t)readings[y1] * 256 | readings[y0];
  dados->z = (dados->z * 256 * 256 * 256) | (int32_t)readings[z2] * 256 * 256 | (uint16_t)readings[z1] * 256 | readings[z0];
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
	HAL_Delay(10);
    HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);

    uint8_t buffer[2];
    buffer[0] = addr & 0x7F;
    buffer[1] = data[0];

    HAL_SPI_Transmit(spi_handle, &buffer[0], 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(spi_handle, &buffer[1], 1, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}
/*
 * addr é o valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser lido
 */
void RM3100_SPI_READ(uint8_t addr, uint8_t *data, uint16_t size)
{
  HAL_Delay(10);
  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET); // digitalWrite(PIN_CS, LOW)
  HAL_Delay(100);                                     // delay(100)
  uint8_t buffer[2];
  buffer[0] = addr | 0x80;
  buffer[1] = 0x00;
  HAL_SPI_Transmit(spi_handle, &buffer[0], 1, 1000);
  HAL_SPI_TransmitReceive(spi_handle, &buffer, data, 1, 1000); // SPI.transfer(addr | 0x80); data = SPI.transfer(0);
  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET);                    // digitalWrite(PIN_CS, HIGH)
  HAL_Delay(10);

}

//newCC is the new cycle count value (16 bits) to change the data acquisition
void changeCycleCount(){
  uint8_t CCMSB = (200 & 0xFF00) >> 8; //get the most significant byte
  uint8_t CCLSB = 200 & 0xFF; //get the least significant byte
  uint revid = 0;

  HAL_Delay(10);
  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);

  uint8_t buffer[7];
  buffer[0] = RM3100_CCX1_REG & 0x7F;
  buffer[1] = CCMSB;
  buffer[2] = CCLSB;
  buffer[3] = CCMSB;
  buffer[4] = CCLSB;
  buffer[5] = CCMSB;
  buffer[6] = CCLSB;
  HAL_SPI_Transmit(spi_handle, &buffer[0], 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(spi_handle, &buffer[1], 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(spi_handle, &buffer[2], 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(spi_handle, &buffer[3], 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(spi_handle, &buffer[4], 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(spi_handle, &buffer[5], 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(spi_handle, &buffer[6], 1, HAL_MAX_DELAY);

  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET);
  HAL_Delay(10);

}

/* Faz a configuração da conexão com o stm32*/
void RM3100_SPI_SETUP(GPIO_InitTypeDef *GPIO_InitStruct)
{
  uint revid = 0;
  RM3100_SPI_READ(RM3100_REVID_REG, &revid, 0);
  RM3100_SPI_READ(RM3100_REVID_REG, &revid, 0);
  printf("REVID ID = 0x%02X\n", revid);

  //changeCycleCount();

  uint16_t cycleCount = 0;
  RM3100_SPI_READ(RM3100_CCX1_REG, &cycleCount, 0);
  RM3100_SPI_READ(RM3100_CCX0_REG, &cycleCount, 0);
  cycleCount = (cycleCount << 8) | cycleCount;
  printf("Cycle Counts = %u\n", cycleCount);

  float gain = (0.3671 * (float)cycleCount) + 1.5;

  if (SINGLE_MODE)
  {
    uint8_t value = 0x00;
    uint16_t teste = 0;

    RM3100_SPI_WRITE(RM3100_CMM_REG, &value, 0);
    RM3100_SPI_WRITE(RM3100_CMM_REG, &value, 0);

    value = 0x70;
    RM3100_SPI_WRITE(RM3100_POLL_REG, &value, 0);
    RM3100_SPI_WRITE(RM3100_POLL_REG, &value, 0);


    RM3100_SPI_READ(RM3100_POLL_REG, &teste, 0);
    RM3100_SPI_READ(RM3100_POLL_REG, &teste, 0);
    teste = teste;
  }
  else
  {
    uint8_t value = 0x00;

	HAL_Delay(1000);
	value = 0b01110000;
	RM3100_SPI_WRITE(RM3100_CMM_REG, &value, 0);
	HAL_Delay(1000);

  }
}

/* Função que deve ficar em loop, retorna uma struct contendo os dados de leitura*/
RM3100_DATA RM3100_SPI_DATA()
{
  RM3100_DATA dados;
  long x = 0;
  long y = 0;
  long z = 0;
  uint8_t buffer[10] = { 0 };
  uint8_t buffers[10] = { 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0x00 };
  uint8_t buffers2[10] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
   // wait_dr();

    HAL_Delay(10);
    HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);

    HAL_SPI_Transmit(spi_handle, &buffers[0], 1, HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(spi_handle, &buffers[1], buffer, 10, HAL_MAX_DELAY);

    HAL_Delay(10);
    HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET);
    HAL_Delay(100);

  //special bit manipulation since there is not a 24 bit signed int data type
  if (buffer[0] & 0x80){
      x = 0xFF;
  }
  if (buffer[3] & 0x80){
      y = 0xFF;
  }
  if (buffer[6] & 0x80){
      z = 0xFF;
  }


  //format results into single 32 bit signed value
  x = (x * 256 * 256 * 256) | (int32_t)(buffer[0]) * 256 * 256 | (uint16_t)(buffer[1]) * 256 | buffer[2];
  y = (y * 256 * 256 * 256) | (int32_t)(buffer[3]) * 256 * 256 | (uint16_t)(buffer[4]) * 256 | buffer[5];
  z = (z * 256 * 256 * 256) | (int32_t)(buffer[6]) * 256 * 256 | (uint16_t)(buffer[7]) * 256 | buffer[8];

  //calculate magnitude of results
  double uT = sqrt(pow(((float)(x)/gain),2) + pow(((float)(y)/gain),2)+ pow(((float)(z)/gain),2));

  return dados;
}
