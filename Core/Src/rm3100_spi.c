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
    while (HAL_GPIO_ReadPin(GPIOB, DR_PIN) == GPIO_PIN_RESET)
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
  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);
  uint8_t buffer[2];
  buffer[0] = addr & 0x7F;
  buffer[1] = data[0];
  HAL_SPI_Transmit(spi_handle, &buffer, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET);
}
/*
 * addr é o valor de 7 bits do endereço do registrador, data é o valor de 8
 * bits referente ao dado a ser lido
 */
void RM3100_SPI_READ(uint8_t addr, uint8_t *data, uint16_t size)
{
  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET); // digitalWrite(PIN_CS, LOW)
  HAL_Delay(100);                                     // delay(100)
  uint8_t buffer[2];
  buffer[0] = addr | 0x80;
  buffer[1] = 0x00;
  HAL_SPI_Transmit(spi_handle, &buffer, 1, HAL_MAX_DELAY);
  HAL_SPI_TransmitReceive(spi_handle, buffer, data, 1, HAL_MAX_DELAY); // SPI.transfer(addr | 0x80); data = SPI.transfer(0);
  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET);                    // digitalWrite(PIN_CS, HIGH)
}

/*
 * RM3100_SPI_CHANGE_CC: Faz a mudança do Cycle Count
 * ----------------------------------------------------------------------
 * new_cc: Novo valor de Cycle Count do rm3100
 */
void RM3100_SPI_CHANGE_CC(uint16_t new_cc)
{
  uint8_t CCMSB = (new_cc & 0xFF00) >> 8; /* pega o byte mais significativo */
  uint8_t CCLSB = new_cc & 0xFF;          /* pega o byte menos significativo */

  uint8_t buffer[] = {
      CCMSB, /* write new cycle count to ccx1 */
      CCLSB, /* write new cycle count to ccx0 */
      CCMSB, /* write new cycle count to ccy1 */
      CCLSB, /* write new cycle count to ccy0 */
      CCMSB, /* write new cycle count to ccz1 */
      CCLSB  /* write new cycle count to ccz0 */
  };

  RM3100_SPI_WRITE(CCX1_REG, buffer, 6);
}

/* Faz a configuração da conexão com o stm32*/
void RM3100_SPI_SETUP(GPIO_InitTypeDef *GPIO_InitStruct)
{
  uint revid = 0;
  RM3100_SPI_READ(RM3100_REVID_REG, &revid, 0);
  RM3100_SPI_READ(RM3100_REVID_REG, &revid, 0);
  printf("REVID ID = 0x%02X\n", revid);


  //	changeCycleCount(initialCC);

  uint16_t cycleCount = 0;
  RM3100_SPI_READ(RM3100_CCX1_REG, &cycleCount, 0);
  RM3100_SPI_READ(RM3100_CCX0_REG, &cycleCount, 0);
  cycleCount = (cycleCount << 8) | cycleCount;
  printf("Cycle Counts = %u\n", cycleCount);

  float gain = (0.3671 * (float)cycleCount) + 1.5;

  if (SINGLE_MODE)
  {
    uint8_t value = 0x00;
    RM3100_SPI_WRITE(RM3100_CMM_REG, &value, 0);
    value = 0x70;
    RM3100_SPI_WRITE(RM3100_POLL_REG, &value, 0);
  }
  else
  {
    uint8_t value = 0x79;
    RM3100_SPI_WRITE(RM3100_CMM_REG, &value, 0);
  }

  //	  uint8_t addr = 0x36;
  //	  uint8_t data = 0;
  ////	  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET); //digitalWrite(PIN_CS, LOW)
  ////	  HAL_Delay(100); //delay(100)
  ////	  uint8_t buffer[2];
  ////	  buffer[0] = addr | 0x80;
  ////	  buffer[1] = 0x00;
  ////	  HAL_SPI_Transmit(spi_handle, &buffer, 1, HAL_MAX_DELAY);
  ////	  HAL_SPI_TransmitReceive(spi_handle, buffer, &data, 1, HAL_MAX_DELAY); //SPI.transfer(addr | 0x80); data = SPI.transfer(0);
  ////	  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET); //digitalWrite(PIN_CS, HIGH)
  //    uint8_t teste = 1;
  //	RM3100_SPI_READ(addr, &data, 0);
  //	teste++;
  //	RM3100_SPI_READ(RM3100_POLL_REG, &data, 0);
  //	teste++;
  //	RM3100_SPI_WRITE(RM3100_POLL_REG, &data,0);
  //	teste++;
  //	RM3100_SPI_READ(RM3100_POLL_REG, &data, 0);
  //	teste++;

  /*Definição de pino Data Ready*/
  //	GPIO_InitStruct->Pin = DR_PIN;
  //	GPIO_InitStruct->Mode = GPIO_MODE_INPUT;
  //	GPIO_InitStruct->Pull = GPIO_NOPULL;
  //	HAL_GPIO_Init(DR_GPIO, GPIO_InitStruct);
  //
  //	/*faz a leitura do Revid e em seguida printa no uart*/
  //	RM3100_SPI_READ(REVID_REG, &revid, 1);
  //
  //	/*deve ser 0x22*/
  //	sprintf(msg, "REVID ID = 0x%d", revid);
  //	uart_print(msg, UART_DBG);
  //
  //	/*Altera o cycle count*/
  //	RM3100_SPI_CHANGE_CC((uint16_t) INITIAL_CC);
  //
  //	uint8_t cc_tmp1, cc_tmp2;
  //	RM3100_SPI_READ(CCX1_REG, &cc_tmp1, 1);
  //	RM3100_SPI_READ(CCX0_REG, &cc_tmp2, 1);
  //
  //	cycle_count = (((uint16_t) cc_tmp1) << 8) | ((uint16_t) cc_tmp2);
  //
  //	sprintf(msg, "Cycles count = %d", cycle_count);
  //	uart_print(msg, UART_DBG);
  //
  //	/* Equação linear para calcular o ganho por meio de cycle_count */
  //	gain = (0.3671 * (float)cycle_count) + 1.5;
  //
  //	sprintf(msg, "Gain = %u.%u", (unsigned int) gain*100 / 100, (unsigned int) gain*100%100);
  //	uart_print(msg, UART_DBG);
  //
  //	uint8_t tmp;
  //	if (SINGLE_MODE)
  //	{
  //		/*Ativa o single mode*/
  //		tmp  = 0;
  //		RM3100_SPI_WRITE(CMM_REG, &tmp, 1);
  //
  //		tmp  = 0x70;
  //		RM3100_SPI_WRITE(POLL_REG, &tmp, 1);
  //	}
  //	else
  //	{
  //		/*Permite a transmissão de medidas contínuas com as funções de alarme desligadas */
  //		tmp = 0x79;
  //		RM3100_SPI_WRITE(CMM_REG, &tmp, 1);
  //	}
}

/* Função que deve ficar em loop, retorna uma struct contendo os dados de leitura*/
RM3100_DATA RM3100_SPI_DATA()
{
  RM3100_DATA dados;
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;
  uint8_t x2, x1, x0, y2, y1, y0, z2, z1, z0;

  // wait until data is ready using 1 of two methods (chosen in options at top of code)
  	uint8_t status_reg;
  	do {
  		RM3100_SPI_READ(RM3100_STATUS_REG, &status_reg, 1); //read internal status register
  	} while ((status_reg & 0x80) != 0x80);

  // read measurements
  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);

  uint8_t buffer2 = 0xA4;

  HAL_SPI_Transmit(spi_handle, &buffer2, 1, HAL_MAX_DELAY);
  uint8_t buffer = 0;
  uint8_t *data;

  // Transfer 0xA5 command and read x-axis data
  buffer = 0xA5;
  HAL_SPI_TransmitReceive(spi_handle, buffer, data, 1, HAL_MAX_DELAY);
  x2 = data[0];
  x1 = data[1];
  x0 = data[2];

  // Transfer 0xA6 command and read y-axis data
  buffer = 0xA6;
  HAL_SPI_TransmitReceive(spi_handle, buffer, data, 1, HAL_MAX_DELAY);
  y2 = data[0];
  y1 = data[1];
  y0 = data[2];

  // Transfer 0xA7 command and read z-axis data
  buffer = 0xA7;
  HAL_SPI_TransmitReceive(spi_handle, buffer, data, 1, HAL_MAX_DELAY);
  z2 = data[0];
  z1 = data[1];
  z0 = data[2];

  // Transfer 0xA8 command and read y-axis data
  buffer = 0xA8;
  HAL_SPI_TransmitReceive(spi_handle, buffer, data, 1, HAL_MAX_DELAY);
  y2 = data[0];
  y1 = data[1];
  y0 = data[2];

  // Transfer 0xA9 command and read y-axis data
  buffer = 0xA9;
  HAL_SPI_TransmitReceive(spi_handle, buffer, data, 3, HAL_MAX_DELAY);
  y2 = data[0];
  y1 = data[1];
  y0 = data[2];

  // Transfer 0xAA command and read y-axis data
  buffer = 0xAA;
  HAL_SPI_TransmitReceive(spi_handle, buffer, data, 1, HAL_MAX_DELAY);
  y2 = data[0];
  y1 = data[1];
  y0 = data[2];

  // Transfer 0xAB command and read z-axis data
  buffer = 0xAB;
  HAL_SPI_TransmitReceive(spi_handle, buffer, data, 3, HAL_MAX_DELAY);
  z2 = data[0];
  z1 = data[1];
  z0 = data[2];

  // Transfer 0xAC command and read z-axis data
  buffer = 0xAC;
  HAL_SPI_TransmitReceive(spi_handle, buffer, data, 1, HAL_MAX_DELAY);
  z2 = data[0];
  z1 = data[1];
  z0 = data[2];

  // Transfer 0x00 command and read z-axis data
  buffer = 0x00;
  HAL_SPI_TransmitReceive(spi_handle, buffer, data, 1, HAL_MAX_DELAY);
  z0 = data[2];

  HAL_GPIO_WritePin(CS_GPIO, CS_PIN, GPIO_PIN_SET);

  // special bit manipulation since there is not a 24 bit signed int data type
  if (x2 & 0x80)
  {
    x = 0xFFFFFF;
  }
  if (y2 & 0x80)
  {
    y = 0xFFFFFF;
  }
  if (z2 & 0x80)
  {
    z = 0xFFFFFF;
  }

  // format results into single 32 bit signed value
  x = (x * 256 * 256 * 256) | ((int32_t)x2 * 256 * 256) | ((uint16_t)x1 * 256) | x0;
  y = (y * 256 * 256 * 256) | ((int32_t)y2 * 256 * 256) | ((uint16_t)y1 * 256) | y0;
  z = (z * 256 * 256 * 256) | ((int32_t)z2 * 256 * 256) | ((uint16_t)z1 * 256) | z0;

  // calculate magnitude of results
  float uT = sqrt(pow(((float)x / gain), 2) + pow(((float)y / gain), 2) + pow(((float)z / gain), 2));

  // display results
  //	printf("Data in counts:   X:%ld   Y:%ld   Z:%ld\n", x, y, z);
  //	printf("Data in microTesla(uT):   X:%f   Y:%f   Z:%f\n", ((float)x / gain), ((float)y / gain), ((float)z / gain));
  //	printf("Magnitude(uT): %f\n\n", uT);

  return dados;
}
