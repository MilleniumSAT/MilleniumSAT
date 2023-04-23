#include "AT24C512C.h"
#include <stdlib.h>
#include <stdio.h>
#include "stm32l0xx_hal.h"
#include <limits.h>
#include <stdio.h>
#include <math.h>
#include <tmp100_i2c.h>

#if (_EEPROM_USE_FREERTOS == 1)
#include "cmsis_os.h"
#define at24_delay(x) osDelay(x)
#else
#define at24_delay(x) HAL_Delay(x)
#endif

#define CRC_POLY 0x1021


#if (_EEPROM_SIZE_KBIT == 1) || (_EEPROM_SIZE_KBIT == 2)
#define _EEPROM_PSIZE 8
#elif (_EEPROM_SIZE_KBIT == 4) || (_EEPROM_SIZE_KBIT == 8) || (_EEPROM_SIZE_KBIT == 16)
#define _EEPROM_PSIZE 16
#else
#define _EEPROM_PSIZE 32
#endif

static uint8_t at24_lock = 0;


uint16_t crc16(uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  Checks if memory device is ready for communication.
 * @param  none
 * @retval bool status
 */
bool at24_isConnected(void)
{
#if (_EEPROM_USE_WP_PIN == 1)
  HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN, GPIO_PIN_SET);
#endif
  if (HAL_I2C_IsDeviceReady(i2c_handle, _EEPROM_ADDRESS, 2, MAX_TIMEOUT_MEM) == HAL_OK)
    return true;
  else
    return false;
}

uint16_t obtainPkgCount(){
	uint16_t qntd = (current_address - pointer_address)/pkt_size;
	return qntd;
}

/**
 * @brief  Write an amount of data in blocking mode to a specific memory address
 * @param  address Internal memory address
 * @param  data Pointer to data buffer
 * @param  len Amount of data to be sent
 * @param  timeout Timeout duration
 * @retval bool status
 */
bool at24_write(uint16_t address, uint8_t *data, size_t len, uint32_t timeout)
{
  if (at24_lock == 1)
    return false;

  at24_lock = 1;
  uint16_t w;
  uint32_t startTime = HAL_GetTick();

#if (_EEPROM_USE_WP_PIN == 1)
  HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN, GPIO_PIN_RESET);
#endif

  while (1)
  {
    w = _EEPROM_PSIZE - (address % _EEPROM_PSIZE);
    if (w > len)
      w = len;
#if ((_EEPROM_SIZE_KBIT == 1) || (_EEPROM_SIZE_KBIT == 2))
    if (HAL_I2C_Mem_Write(i2c_handle, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, w, MAX_TIMEOUT_MEM) == HAL_OK)
#elif (_EEPROM_SIZE_KBIT == 4)
    if (HAL_I2C_Mem_Write(i2c_handle, _EEPROM_ADDRESS | ((address & 0x0100) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, w, MAX_TIMEOUT_MEM) == HAL_OK)
#elif (_EEPROM_SIZE_KBIT == 8)
    if (HAL_I2C_Mem_Write(i2c_handle, _EEPROM_ADDRESS | ((address & 0x0300) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, w, MAX_TIMEOUT_MEM) == HAL_OK)
#elif (_EEPROM_SIZE_KBIT == 16)
    if (HAL_I2C_Mem_Write(i2c_handle, _EEPROM_ADDRESS | ((address & 0x0700) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, w, MAX_TIMEOUT_MEM) == HAL_OK)
#else
    if (HAL_I2C_Mem_Write(i2c_handle, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, data, w, MAX_TIMEOUT_MEM) == HAL_OK)
#endif
    {
      at24_delay(10);
      len -= w;
      data += w;
      address += w;
      if (len == 0)
      {
#if (_EEPROM_USE_WP_PIN == 1)
        HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN, GPIO_PIN_SET);
#endif
        at24_lock = 0;
        return true;
      }
      if (HAL_GetTick() - startTime >= timeout)
      {
        at24_lock = 0;
        return false;
      }
    }
    else
    {
#if (_EEPROM_USE_WP_PIN == 1)
      HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN, GPIO_PIN_SET);
#endif
      at24_lock = 0;
      return false;
    }
  }
}

/**
 * @brief  Read an amount of data in blocking mode to a specific memory address
 * @param  address Internal memory address
 * @param  data Pointer to data buffer
 * @param  len Amount of data to be sent
 * @param  timeout Timeout duration
 * @retval bool status
 */
bool at24_read(uint16_t address, uint8_t *data, size_t len, uint32_t timeout)
{
  if (at24_lock == 1)
    return false;
  at24_lock = 1;
#if (_EEPROM_USE_WP_PIN == 1)
  HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN, GPIO_PIN_SET);
#endif
#if ((_EEPROM_SIZE_KBIT == 1) || (_EEPROM_SIZE_KBIT == 2))
  if (HAL_I2C_Mem_Read(i2c_handle, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, len, MAX_TIMEOUT_MEM) == HAL_OK)
#elif (_EEPROM_SIZE_KBIT == 4)
  if (HAL_I2C_Mem_Read(i2c_handle, _EEPROM_ADDRESS | ((address & 0x0100) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, len, MAX_TIMEOUT_MEM) == HAL_OK)
#elif (_EEPROM_SIZE_KBIT == 8)
  if (HAL_I2C_Mem_Read(i2c_handle, _EEPROM_ADDRESS | ((address & 0x0300) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, len, MAX_TIMEOUT_MEM) == HAL_OK)
#elif (_EEPROM_SIZE_KBIT == 16)
  if (HAL_I2C_Mem_Read(i2c_handle, _EEPROM_ADDRESS | ((address & 0x0700) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, len, MAX_TIMEOUT_MEM) == HAL_OK)
#else
  if (HAL_I2C_Mem_Read(i2c_handle, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, data, len, timeout) == HAL_OK)
#endif
  {
    at24_lock = 0;
    return true;
  }
  else
  {
    at24_lock = 0;
    return false;
  }
}

/**
 * @brief  Erase memory.
 * @param  none
 * @retval bool status
 */
bool at24_eraseChip(void)
{
	const uint8_t eraseData[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	 uint32_t bytes = 0;
  while (bytes < (_EEPROM_SIZE_KBIT * 128))
  {
    if (at24_write(bytes, (uint8_t *)eraseData, sizeof(eraseData), MAX_TIMEOUT_MEM) == false)
      return false;
    bytes += sizeof(eraseData);
  }
  return true;
}

enum ERROR write_data_to_eeprom(TMP100_DATA *tmp_data, RM3100_DATA *rm_data)
{
  // Write ID value to EEPROM
  at24_write(current_address, (uint8_t *)&id_pkt, sizeof(id_pkt), MAX_TIMEOUT_MEM);
  current_address += sizeof(id_pkt);
  id_pkt++;

  // Write temperature value to EEPROM
  at24_write(current_address, (uint8_t *)&tmp_data->temp, sizeof(tmp_data->temp), MAX_TIMEOUT_MEM);
  current_address += sizeof(tmp_data->temp);

  // Calculate and write CRC of temperature value to EEPROM
  uint16_t crc_temp = crc16((uint8_t *)&tmp_data->temp, sizeof(tmp_data->temp));
  at24_write(current_address, (uint8_t *)&crc_temp, sizeof(crc_temp), MAX_TIMEOUT_MEM);
  current_address += sizeof(crc_temp);

  // Write status value to EEPROM
  at24_write(current_address, (uint8_t *)&tmp_data->status, sizeof(tmp_data->status), MAX_TIMEOUT_MEM);
  current_address += sizeof(tmp_data->status);

  // Calculate and write CRC of status value to EEPROM
  uint16_t crc_status = crc16((uint8_t *)&tmp_data->status, sizeof(tmp_data->status));
  at24_write(current_address, (uint8_t *)&crc_status, sizeof(crc_status), MAX_TIMEOUT_MEM);
  current_address += sizeof(crc_status);

  // Write x, y, and z values to EEPROM
  at24_write(current_address, (uint8_t *)&rm_data->x, sizeof(rm_data->x), MAX_TIMEOUT_MEM);
  current_address += sizeof(rm_data->x);
  at24_write(current_address, (uint8_t *)&rm_data->y, sizeof(rm_data->y), MAX_TIMEOUT_MEM);
  current_address += sizeof(rm_data->y);
  at24_write(current_address, (uint8_t *)&rm_data->z, sizeof(rm_data->z), MAX_TIMEOUT_MEM);
  current_address += sizeof(rm_data->z);

  // Calculate and write CRC of x, y, and z values to EEPROM
  uint16_t crc_xyz = crc16((uint8_t *)&rm_data->x, sizeof(rm_data->x) * 3);
  at24_write(current_address, (uint8_t *)&crc_xyz, sizeof(crc_xyz), MAX_TIMEOUT_MEM);
  current_address += sizeof(crc_xyz);

  // Write gain value to EEPROM
  at24_write(current_address, (uint8_t *)&rm_data->gain, sizeof(rm_data->gain), MAX_TIMEOUT_MEM);
  current_address += sizeof(rm_data->gain);

  // Calculate and write CRC of gain value to EEPROM
  uint16_t crc_gain = crc16((uint8_t *)&rm_data->gain, sizeof(rm_data->gain));
  at24_write(current_address, (uint8_t *)&crc_gain, sizeof(crc_gain), MAX_TIMEOUT_MEM);
  current_address += sizeof(crc_gain);

  // Write uT value to EEPROM
  at24_write(current_address, (uint8_t *)&rm_data->uT, sizeof(rm_data->uT), MAX_TIMEOUT_MEM);
  current_address += sizeof(rm_data->uT);

  // Calculate and write CRC of uT value to EEPROM
  uint16_t crc_ut = crc16((uint8_t *)&rm_data->uT, sizeof(rm_data->uT));
  at24_write(current_address, (uint8_t *)&crc_ut, sizeof(crc_ut), MAX_TIMEOUT_MEM);
  current_address += sizeof(crc_ut);

  return ERROR_OK;
}

enum ERROR read_data_from_eeprom(TMP100_DATA *tmp_data, RM3100_DATA *rm_data)
{
  // Read ID value from EEPROM
  at24_read(pointer_address, (uint8_t *)&tmp_data->id, sizeof(tmp_data->id), MAX_TIMEOUT_MEM);
  rm_data->id = tmp_data->id;
  pointer_address += sizeof(tmp_data->id);

  // Read temperature value from EEPROM
  at24_read(pointer_address, (uint8_t *)&tmp_data->temp, sizeof(tmp_data->temp), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(tmp_data->temp);

  // Read and verify CRC of temperature value from EEPROM
  uint16_t crc_temp = 0;
  at24_read(pointer_address, (uint8_t *)&crc_temp, sizeof(crc_temp), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(crc_temp);
  if (crc_temp != crc16((uint8_t *)&tmp_data->temp, sizeof(tmp_data->temp)))
  {
    // Handle error
    return ERROR_FAIL;
  }

  // Read status value from EEPROM
  at24_read(pointer_address, (uint8_t *)&tmp_data->status, sizeof(tmp_data->status), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(tmp_data->status);

  // Read and verify CRC of status value from EEPROM
  uint16_t crc_status = 0;
  at24_read(pointer_address, (uint8_t *)&crc_status, sizeof(crc_status), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(crc_status);
  if (crc_status != crc16((uint8_t *)&tmp_data->status, sizeof(tmp_data->status)))
  {
    // Handle error
    return ERROR_FAIL;
  }

  // Read x, y, and z values from EEPROM
  at24_read(pointer_address, (uint8_t *)&rm_data->x, sizeof(rm_data->x), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(rm_data->x);
  at24_read(pointer_address, (uint8_t *)&rm_data->y, sizeof(rm_data->y), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(rm_data->y);
  at24_read(pointer_address, (uint8_t *)&rm_data->z, sizeof(rm_data->z), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(rm_data->z);

  // Read and verify CRC of x, y, and z values from EEPROM
  uint16_t crc_xyz = 0;
  at24_read(pointer_address, (uint8_t *)&crc_xyz, sizeof(crc_xyz), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(crc_xyz);
  if (crc_xyz != crc16((uint8_t *)&rm_data->x, sizeof(rm_data->x) * 3))
  {
    // Handle error
    return ERROR_FAIL;
  }

  // Read gain value from EEPROM
  at24_read(pointer_address, (uint8_t *)&rm_data->gain, sizeof(rm_data->gain), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(rm_data->gain);

  // Read and verify CRC of gain value from EEPROM
  uint16_t crc_gain = 0;
  at24_read(pointer_address, (uint8_t *)&crc_gain, sizeof(crc_gain), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(crc_gain);
  if (crc_gain != crc16((uint8_t *)&rm_data->gain, sizeof(rm_data->gain)))
  {
    // Handle error
    return ERROR_FAIL;
  }

  // Read gain value from EEPROM
  at24_read(pointer_address, (uint8_t *)&rm_data->uT, sizeof(rm_data->uT), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(rm_data->uT);

  // Read and verify CRC of gain value from EEPROM
  uint16_t crc_ut = 0;
  at24_read(pointer_address, (uint8_t *)&crc_ut, sizeof(crc_ut), MAX_TIMEOUT_MEM);
  pointer_address += sizeof(crc_ut);
  if (crc_ut != crc16((uint8_t *)&rm_data->uT, sizeof(rm_data->uT)))
  {
    // Handle error
    return ERROR_FAIL;
  }

  // Success
  return ERROR_OK;
}

/* EOF */
