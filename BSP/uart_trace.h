//#include "stm32f4xx_hal.h"

#ifdef TRACE_UART

//int uart_vprintf(const char *msg, va_list ap);
int uart_printf(const char *msg, ...);

#else

#define uart_vprintf(...) (void)0
#define uart_printf(...)	(void)0

#endif

