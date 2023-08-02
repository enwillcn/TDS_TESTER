#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tds_display.h"
#include "freertos/queue.h"

#define TXD_PIN (GPIO_NUM_8)
#define RXD_PIN (GPIO_NUM_9)

#define TXD_PIN_2 (GPIO_NUM_6)
#define RXD_PIN_2 (GPIO_NUM_7)


/**********************
 *      TYPEDEFS
 **********************/


/**********************
 *  GLOBAL VARIABLES
 **********************/
QueueHandle_t tdsQueue; //hold the tds and filter timer values

/**********************
 *  STATIC VARIABLES
 **********************/
static const int RX_BUF_SIZE = 1024;
char tx_cmd[15] = {0xA0, 0x00, 0x00, 0x00, 0x00, 0xA0};



/**********************
 *  STATIC PROTOTYPES
 **********************/
static int sendData(uart_port_t uartChannel, const char* data);



/**********************
 *   STATIC FUNCTIONS
 **********************/

static int sendData(uart_port_t uartChannel, const char* data)
{
    //const int len = strlen(data);
    const int txBytes = uart_write_bytes(uartChannel, data, 6);
    ESP_LOGI("TX", "Wrote %d bytes", txBytes);
    return txBytes;
}

/**
 * @brief uart send task, send command to tds sensor every 2 seconds
 * 
 * @param arg 
 */
static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(UART_NUM_1, tx_cmd);
        vTaskDelay(pdMS_TO_TICKS(20));
        sendData(UART_NUM_2, tx_cmd);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/**
 * @brief UART receive task
 * 
 * @param arg 
 */
static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, pdMS_TO_TICKS(500));
        if (rxBytes == 6) {
            if(data[0] == 0xaa)
            {
                // todo
            }
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes from channel 1: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            uint32_t tds1 = 0;
            tds1 = data[1]<<8 | data[2];
            tdsData_t data1 ={0};
            data1.dataSource = IN_TDS;
            data1.value = tds1;
            xQueueSend(tdsQueue, &data1, 0);
        }
        
        rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, pdMS_TO_TICKS(500));
        if (rxBytes == 6) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes from channel 2: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            uint32_t tds2 = 0;
            tds2 = data[1]<<8 | data[2];
            tdsData_t data2 ={0};
            data2.dataSource = OUT_TDS;
            data2.value = tds2;
            xQueueSend(tdsQueue, &data2, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    free(data);
}





/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void tds_uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN_2, RXD_PIN_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    tdsQueue = xQueueCreate(6, sizeof(tdsData_t));

    if(tdsQueue != NULL){
        xTaskCreate(rx_task, "uart_rx_task", 1024*5, NULL, configMAX_PRIORITIES, NULL);
        xTaskCreate(tx_task, "uart_tx_task", 1024*5, NULL, configMAX_PRIORITIES-1, NULL);
    }

}













