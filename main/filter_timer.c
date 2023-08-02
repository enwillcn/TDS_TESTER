/**
 * @file filter_timer.c
 * @author Will
 * @brief this file contains the functions that are used to update the filter timer every 4hours
 *        using the FreeRTOS software timer and esp NVS to store the filter service days
 * @version 0.1
 * @date 2023-07-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/*********************
 *      INCLUDES
 *********************/
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "tds_checking.h"
#include "tds_display.h"

/*********************
 *      DEFINES
 *********************/

#define FILTER_TIMER_PERIOD_MS          24*60*60*1000     // !24hours every timer cb, This method has overflow issue!
#define FILTER_TIMER_PERIOD_TICKS       24*60*60*1000   // 24hours every timer cb
#define FILTER_NVS_NAME             "filter_days"

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void filter_timer_cb( TimerHandle_t xTimer );


/**********************
 *  GLOBAL VARIABLES
 **********************/
QueueHandle_t filterDaysQueueHandle;


/**********************
 *  STATIC VARIABLES
 **********************/
static TimerHandle_t filter_timer;
static int32_t filter_service_days = 0;
static BaseType_t timerStarted;
/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}

/**
 * @brief read filter service days from NVS and then start the filter timer
 * 
 * @return
 */
void filter_read_nvs_on_restart(void)
{
    nvs_handle_t my_handle;
    //int days = 0; // value will default to 0, if not set yet in NVS
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } 
    else {
        // Read data from NVS handle
        printf("Reading filter service days from NVS ... ");
        err = nvs_get_i32(my_handle, FILTER_NVS_NAME, &filter_service_days);
        switch (err) {
            case ESP_OK:
                printf("filter service days = %d\n", (int)filter_service_days);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

    }
    nvs_close(my_handle);

    filter_timer = xTimerCreate(
                "filter_service_timer",
                //pdMS_TO_TICKS(FILTER_TIMER_PERIOD_MS),
                FILTER_TIMER_PERIOD_TICKS,
                pdTRUE,
                0,
                filter_timer_cb);
    xTimerStart(filter_timer, 0);
    // if(filter_timer != NULL){
    //     timerStarted = xTimerStart(filter_timer, 0);
    // }

    // filterDaysQueueHandle = xQueueCreate(1, sizeof(uint32_t));
    // xQueueSend(filterDaysQueueHandle, &filter_service_days, 0);

        tdsData_t filterDays = { 0 };
        filterDays.dataSource = SERVE_DAYS;
        filterDays.value = filter_service_days;
        xQueueSend(tdsQueue, &filterDays, 0);

}

/**
 * @brief reset filter timer days in nvs after filer service
 * 
 */
void filter_reset_days(void)
{
    filter_service_days = 0;
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    err = nvs_set_i32(my_handle, FILTER_NVS_NAME, filter_service_days);
    err = nvs_commit(my_handle);
    printf("filter service days = %d\n", (int)filter_service_days);
    nvs_close(my_handle);
    tdsData_t filterDays = { 0 };
    filterDays.dataSource = SERVE_DAYS;
    filterDays.value = filter_service_days;
    xQueueSend(tdsQueue, &filterDays, 0);
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void filter_timer_cb( TimerHandle_t xTimer )
{
    nvs_handle_t my_handle;
    printf("Updating filter service days in NVS ... ");
    filter_service_days++;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    err = nvs_set_i32(my_handle, FILTER_NVS_NAME, filter_service_days);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    printf("filter service days = %d\n", (int)filter_service_days);

    // Close
    nvs_close(my_handle);
    //xQueueSend(filterDaysQueueHandle, &filter_service_days, 0);
    tdsData_t filterDays = { 0 };
    filterDays.dataSource = SERVE_DAYS;
    filterDays.value = filter_service_days;
    xQueueSend(tdsQueue, &filterDays, 0);
}