#include "stm32wbxx_hal.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include "uart_trace.h"


#ifdef TRACE_UART
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef hlpuart1;

static volatile int InUsed=0;
static char uart_buffer[256];
static uint32_t UartErrCnt=0; 

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    // TODO check if any more to send and do it
    InUsed=0; 
}

int uart_vprintf(const char *msg, va_list ap){ 
    int n;
    int status;
    while( InUsed ){
           //
        __WFI();
    } 
    InUsed|=1;
    n=vsnprintf(uart_buffer, sizeof(uart_buffer),  msg, ap);
    status = HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)uart_buffer, n );
    if( status ){
        UartErrCnt++;
        InUsed=0;
    }
    return n;
}

int uart_printf(const char *msg, ...){
	va_list ap;
    int n;
    while( InUsed ){
        //
        __WFI();
    }
    va_start(ap, msg);
    n=uart_vprintf(msg, ap);
    va_end(ap);
    return n;
}

#else
#define uart_vprintf(...) (void)0
#define uart_printf(...)	(void)0
#endif

