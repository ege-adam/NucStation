/*
 * GameFlasher.h
 *
 */

#ifndef INC_GAMEFLASHER_H_
#define INC_GAMEFLASHER_H_

#include "stm32l0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#define ETX_APP_FLASH_ADDR 0x08008000   //Application's Flash Address

HAL_StatusTypeDef FlashTheGame( uint32_t *data, uint16_t data_sector, bool is_first_block);

#endif /* INC_GAMEFLASHER_H_ */
