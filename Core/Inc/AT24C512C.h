#ifndef	_AT24CXX_H
#define	_AT24CXX_H

#include "stm32l0xx_hal.h"
#include "globalvar.h"

/*
  Author:     Suman Kumar D.
  WebSite:    https://github.com/sumancvb
  Version:    1.0.0
*/

/********************************* Configuration Start *********************************/
#define		_EEPROM_SIZE_KBIT		512       /* 256K (32,768 x 8) */
#define		_EEPROM_USE_FREERTOS		0
#define		_EEPROM_ADDRESS			0xA0
#define		_EEPROM_USE_WP_PIN		0
#define		_EEPROM_USE_IWDG		0
#define     START_ADDRESS           0x00
#define     MAX_TIMEOUT_MEM         100

/* The AT24CXX has a hardware data protection scheme that allows the
user to write protect the whole memory when the WP pin is at VCC. */

#if (_EEPROM_USE_WP_PIN == 1)
#define		_EEPROM_WP_GPIO			WP_GPIO_Port
#define		_EEPROM_WP_PIN			WP_Pin
#endif

/* For watchdog timer refresh, required during erase operation */
#if (_EEPROM_USE_IWDG == 1)
#define		_EEPROM_IWDG			hiwdg
#endif

/********************************* Configuration End *********************************/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

uint16_t current_address = START_ADDRESS;
uint16_t pointer_address = START_ADDRESS;

/**
  * @brief  Checks if memory device is ready for communication.
  * @param  none
  * @retval bool
  */
bool at24_isConnected(void);

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  address Internal memory address
  * @param  data Pointer to data buffer
  * @param  len Amount of data to be sent
  * @param  timeout Timeout duration
  * @retval bool status
  */
bool at24_write(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);

/**
  * @brief  Read an amount of data in blocking mode to a specific memory address
  * @param  address Internal memory address
  * @param  data Pointer to data buffer
  * @param  len Amount of data to be sent
  * @param  timeout Timeout duration
  * @retval bool status
  */
bool at24_read(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);

/**
  * @brief  Erase memory.
  * @note   This requires time in seconds
  * @param  none
  * @retval bool status
  */
bool at24_eraseChip(void);

enum ERROR write_data_to_eeprom(TMP100_DATA *tmp_data, RM3100_DATA *rm_data);

enum ERROR read_data_from_eeprom(TMP100_DATA *tmp_data, RM3100_DATA *rm_data);

uint16_t obtainPkgCount();

#ifdef __cplusplus
}
#endif

#endif

/* EOF */
