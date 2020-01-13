#ifndef HI_PARSER_H_
#define HI_PARSER_H_

#ifdef __cplusplus 
extern "C" { 
#endif

#include "simple.pb-c.h"
#include "stm32wbxx_hal.h"

//#include "stm32f4xx_hal.h"

// #include "Fifo.h"
// #include "uart_trace.h"
// extern volatile uint8_t uart_data[MAX_BLE_FIFO_SIZE];

#pragma pack(push, 1) 
struct ModernProtocolWithSTM {
    uint8_t sig1;
    uint8_t sig2;
    uint8_t crc;
    uint32_t length;
    uint8_t data[200];
}; 

#pragma pack(pop)
typedef void (*HiCallbackType)(const ToSt*);
typedef void (*ParseCallbackType)(uint8_t*, size_t);


uint8_t addToBleParserFromHi(uint8_t t) ; 
uint8_t parseBleFromHi(ParseCallbackType p) ;

void pushToAndroid(const ToHost* d) ;

uint8_t addToBleParserFromAndroid(uint8_t t) ; 
uint8_t parseBleFromAndroid(ParseCallbackType p) ;


void pushToHi(UART_HandleTypeDef* uart, const ToHi* d);


#ifdef __cplusplus
}
#endif

#endif
