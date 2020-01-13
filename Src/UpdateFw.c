#include "UpdateFw.h"
#include "flash_if.h"
//#include "circular-buffer.h"

extern UART_HandleTypeDef huart1;

__IO uint32_t flashdestination;
uint64_t filesize;

const uint32_t crc32_tab[] = 
{
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
  0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
  0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
  0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
  0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
  0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
  0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
  0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
  0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
  0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
  0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
  0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
  0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
  0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
  0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
  0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
  0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
  0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
  0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
  0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
  0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
  0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
  0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
  0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
  0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
  0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
  0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
  0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
  0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
  0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
  0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
  0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
  0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

uint32_t flash_crc32(uint32_t starAddr, uint32_t endAddr)
{
  const uint8_t *p = (uint8_t *) starAddr;
  uint32_t size = endAddr - starAddr;
  uint32_t crc = 0;

  crc = ~crc;

  while (size--)
  {
      crc = crc32_tab[(crc ^ *p++) & 0xff] ^ (crc >> 8);
  }

  return crc ^ ~0U;
}

uint64_t Read_Flash(uint32_t Address)
{
    return (*((volatile uint64_t*)(Address)));
}


extern void Update_Fw_Start()
{
  FLASH_If_Erase(FREE_AREA_START_PAGE);
}

extern uint32_t Update_Fw_Data(uint32_t startOffset, uint32_t endOffset, uint8_t *data, uint32_t len)
{
  uint32_t parse_len = len;
  uint32_t flash_crc = 0;

  flashdestination = FREE_AREA_ADDRESS + startOffset;

  if(parse_len % 8 != 0) // remain bytes writes, dont care a memory address over
  {
    parse_len += 8;
  }

  if (FLASH_If_Write(flashdestination, (uint64_t*) data, parse_len / sizeof(uint64_t)) == FLASHIF_OK)
  {
    flash_crc = flash_crc32(FREE_AREA_ADDRESS + startOffset, FREE_AREA_ADDRESS + endOffset);
  }

  filesize = FREE_AREA_ADDRESS + endOffset;

  return flash_crc;

}

extern uint32_t Update_Fw_Verify(uint32_t startOffset, uint32_t endOffset)
{
  uint32_t flash_crc;

  flash_crc = flash_crc32(FREE_AREA_ADDRESS + startOffset, FREE_AREA_ADDRESS + endOffset);

  return flash_crc;
}

extern void Update_Fw_End()
{
  uint64_t flash_flag = FW_UPDATE_FLAG;
  uint64_t flash_crc;
  //uint64_t user_return = FW_FLASH_ERR;
  uint32_t SectorError;

  FLASH_EraseInitTypeDef pEraseInit;

  /* Unlock the Flash to enable the flash control register access *************/ 
  FLASH_If_Init();
  
  /* Get the sector where start the user flash area */
  //UserStartSector = StartSector;//GetSector(APPLICATION_ADDRESS);
  
  pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  pEraseInit.Page = 4;
  pEraseInit.NbPages= 1;

  if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
  {
     /* Error occurred while page erase */
     //return (1);
     return;
  }


  flash_crc = (uint64_t)flash_crc32(FREE_AREA_ADDRESS, filesize);
  
  filesize -= FREE_AREA_ADDRESS;

  FLASH_If_Write(FW_UPDATE_SUCCESS_FLAG_ADDRESS, &flash_flag, 1);
  FLASH_If_Write(FW_UPDATE_SIZE_ADDRESS, &filesize, 1);
  FLASH_If_Write(FW_UPDATE_CRC_ADDRESS, &flash_crc, 1);
  //FLASH_If_Write(FW_UPDATE_RETURN_ADDRESS, &user_return, 1);
}

void Check_Return()
{
    uint64_t get_return = 0;
    get_return = (uint64_t)Read_Flash(FW_UPDATE_RETURN_ADDRESS);

    if(get_return == FW_SIZE_ERR)
    {
        //size err
    }
    else if(get_return == FW_CRC_ERR)
    {
        //crc err
    }
    else if(get_return == FW_FLASH_ERR)
    {
        //flash err
    }    
    else if(get_return == FW_FLAG_ERR || get_return == FW_NONE_ERR)
    {
        //err none
    }
}


#if 0
extern void Update_Fw()
{
    uint32_t flash_flag = BOOTLOADER;
    uint32_t flash_err;
    FLASH_EraseInitTypeDef flash_config;
    uint32_t flash_crc;
    uint32_t data_packet_len;
#if 0
    char *Buf; 
    Buf = RetData();
#endif
    fw_update_mode = Buf[7];
      switch(fw_update_mode)
      {
      case FW_UPDATE_START:
        file_rec_size = 0;
        flashdestination = APPLICATION_ADDRESS;
        bData_rec_error = 0;

        //FLASH_If_Init();
        HAL_FLASH_Unlock();
        HAL_FLASH_OB_Unlock();

        /* erase user application area */
        //FLASH_If_Erase(APPLICATION_ADDRESS);
        flash_config.TypeErase = FLASH_TYPEERASE_PAGES;
        flash_config.Page = USER_APP_START_PAGE;
        flash_config.NbPages = USER_APP_PAGE_NUMBER;
        HAL_FLASHEx_Erase(&flash_config, &flash_err);

        break;
      case FW_UPDATE_END:
        FLASH_If_Write(FW_UPDATE_SUCCESS_FLAG_ADDRESS, &flash_flag, 1);
        FLASH_If_Write(FW_UPDATE_SIZE_ADDRESS, &file_rec_size, 1);
        FLASH_If_Write(FW_UPDATE_CRC_ADDRESS, &verify_flash_crc, 1);

        Buf[0] = 0xCA; // header1
        Buf[1] = 0xB7; // header2
        Buf[2] = 0x55; // cmd
        Buf[3] = 0x00; //length2
        Buf[4] = 0x00; //length1
        Buf[5] = 0x02; //length0
        Buf[6] = 0x00; //firmware id : 0 : stm
        Buf[7] = 0x01; //update mode : 1 : end

        //Send_Ble(&Buf[0],8);

        //JumpToApplication();
    		HAL_UART_Transmit(&huart1, "end", 3, 100);
        break;
      case FW_UPDATE_DATA:
        data_start_offset = (Buf[8] << 24) + (Buf[9] << 16) + (Buf[10] << 8) + (Buf[11]);
        data_end_offset = (Buf[12] << 24) + (Buf[13] << 16) + (Buf[14] << 8) + (Buf[15]) + 1;
        data_rec_size = 0;

        if (FLASH_If_Write(flashdestination, (uint32_t*) &Buf[16], 1) == FLASHIF_OK)
        {
          data_packet_len = (Buf[4] << 8) + Buf[5];
          if(data_packet_len > 16 && data_packet_len < 20)
          {
            flashdestination += Buf[5] - 16;
            data_rec_size += Buf[5] - 16;
            file_rec_size += Buf[5] - 16;
          }
          else
          {
            flashdestination += 4;
            data_rec_size += 4;
            file_rec_size += 4;
          }

	        if(data_start_offset + data_rec_size == data_end_offset)
	        {
	          flash_crc = flash_crc32(APPLICATION_ADDRESS + data_start_offset, APPLICATION_ADDRESS + data_end_offset);

	          Buf[0] = 0xCA; // header1
	          Buf[1] = 0xB7; // header2
	          Buf[2] = 0x55; // cmd
	          Buf[3] = 0x00; //length2
	          Buf[4] = 0x00; //length1
	          Buf[5] = 0x07; //length0
	          Buf[6] = 0x00; //firmware id : 0 : stm
	          Buf[7] = 0x02; //update mode : 2 : data

	          Buf[8] = bData_rec_error; // status

	          Buf[9] = (flash_crc >> 24);
	          Buf[10] = (flash_crc >> 16);
	          Buf[11] = (flash_crc >> 8);
	          Buf[12] = flash_crc;

	          //Send_Ble(&Buf[0],13);

						//memset((void *)BleDataBuf, 0, sizeof(BleDataBuf));
	        }

        }
        else
        {
          bData_rec_error = 1;
        }

        break;
      case FW_UPDATE_VERIFY:
        data_start_offset = (Buf[8] << 24) + (Buf[9] << 16) + (Buf[10] << 8) + (Buf[11]);
        data_end_offset = (Buf[12] << 24) + (Buf[13] << 16) + (Buf[14] << 8) + (Buf[15]) + 1;

        verify_flash_crc = flash_crc32(APPLICATION_ADDRESS + data_start_offset, APPLICATION_ADDRESS + data_end_offset);

        Buf[0] = 0xCA; // header1
        Buf[1] = 0xB7; // header2
        Buf[2] = 0x55; // cmd
        Buf[3] = 0x00; //length2
        Buf[4] = 0x00; //length1
        Buf[5] = 0x07; //length0
        Buf[6] = 0x00; //firmware id : 0 : stm
        Buf[7] = 0x03; //update mode : 3 : verify

        Buf[8] = bData_rec_error; // status

        Buf[9] = (verify_flash_crc >> 24);
        Buf[10] = (verify_flash_crc >> 16);
        Buf[11] = (verify_flash_crc >> 8);
        Buf[12] = verify_flash_crc;

        //Send_Ble(&Buf[0],13);

        break;
    }
}
#endif
