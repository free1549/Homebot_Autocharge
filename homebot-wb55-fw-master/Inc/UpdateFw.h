#ifndef __UPDATE_FW__
#define __UPDATE_FW__

#include "stm32wbxx_hal.h"

#define FREE_AREA_START_PAGE                   37
#define FREE_AREA_PAGE_NUMBER                  46
#define FREE_AREA_FLASH_END_ADDRESS           0x08053000
#define FW_UPDATE_SUCCESS_FLAG_ADDRESS        0x08004000
#define FW_UPDATE_SIZE_ADDRESS                0x08004008
#define FW_UPDATE_CRC_ADDRESS                 0x08004010
#define FW_UPDATE_RETURN_ADDRESS              0x08004018


#define FW_FLASH_ERR        0xFFFFFFFFFFFF
#define FW_FLAG_ERR         0x0000AA0000AA
#define FW_SIZE_ERR         0x0000BB0000BB
#define FW_CRC_ERR          0x0000CC0000CC
#define FW_NONE_ERR         0x000000000000

#define FREE_AREA_ADDRESS   0x08025000
#define USER_DATA_OFFSET    0x0004000
#define FW_UPDATE_FLAG      0xAA55A5A5AA55A5A5

extern void Update_Fw();
extern void Update_Fw_Start();
extern uint32_t Update_Fw_Data(uint32_t startOffset, uint32_t endOffset, uint8_t *data, uint32_t len);
extern uint32_t Update_Fw_Verify(uint32_t startOffset, uint32_t endOffset);
extern void Update_Fw_End();
void Check_Return();


#endif
