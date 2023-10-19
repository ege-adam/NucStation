/*
 * GameFlasher.c
 *
 */

#include "GameFlasher.h"
#include <stdio.h>

/**
  * @brief Write data to the Application's actual flash location.
  * @param data data to be written
  * @param data_len data length
  * @is_first_block true - if this is first block, false - not first block
  * @retaval HAL_StatusTypeDef
  */
HAL_StatusTypeDef FlashTheGame( uint32_t *data, uint16_t data_sector, bool is_first_block)
{
  HAL_StatusTypeDef reta;
  do
  {
    reta = HAL_FLASH_Unlock();
    if( reta != HAL_OK )
    {
      break;
    }
    //No need to erase every time. Erase only the first time.
    if( is_first_block )
    {
      printf("Erasing the Flash memory...\r\n");
      //Erase the Flash
      uint32_t errory = 0;
      FLASH_EraseInitTypeDef EraseInitStruct;

      EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.PageAddress   = 0x8008000;
      EraseInitStruct.NbPages     = 1280;
      reta = HAL_FLASHEx_Erase(&EraseInitStruct, &errory);

      if( reta != HAL_OK )
      {
    	printf("Error...\r\n");
        break;
      }
    }


    uint32_t _data = *data;
    uint32_t adress = (ETX_APP_FLASH_ADDR + (data_sector * 4));
    reta = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, adress, _data);
    if( reta == HAL_OK )
    {
      //update the data count

    }
    else
    {
      printf("Flash Write Error\r\n");
      break;
    }

    reta = HAL_FLASH_Lock();
    if( reta != HAL_OK )
    {
      break;
    }
  }while( false );
  return reta;
}
