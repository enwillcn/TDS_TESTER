#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifndef __TDS_CHECKING__
#define __TDS_CHECKING__



extern QueueHandle_t tdsQueue; //hold the tds and filter timer values

void tds_uart_init(void);



#endif //_TDS_CHECKING__