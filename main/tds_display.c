#include "lvgl.h"
#include "tds_checking.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "freertos/semaphore.h"
#include "freertos/queue.h"
#include "tds_display.h"
#include "filter_timer.h"

/**********************
 *      TYPEDEFS
 **********************/



/**********************
 *  GLOBAL VARIABLES
 **********************/
LV_IMG_DECLARE(filter_gif_160x160);


/**********************
 *  STATIC VARIABLES
 **********************/
static lv_obj_t *tds_screen;                 // screen to hold the main display
static lv_obj_t *filter_reset_screen;        // filter reset screen

static lv_obj_t *btn_cancel;
static lv_obj_t *btn_OK;


lv_obj_t *lable_incomming_tds;
lv_obj_t *lable_purifed_tds;
static lv_obj_t *lable_days_num;


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void tds_filter_num_click_cb(lv_event_t * e);
static void reset_event_hanlder(lv_event_t * e);


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void tds_filter_num_click_cb(lv_event_t * e)
{
    lv_scr_load(filter_reset_screen);
}

static void reset_event_hanlder(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if(obj == btn_cancel){
        lv_scr_load(tds_screen);

    }
    if(obj == btn_OK){
        lv_scr_load(tds_screen);
        tdsData_t filterDays = { 0 };
        filterDays.dataSource = SERVE_DAYS;
        filterDays.value = 0;
        lvgl_update_label_number(filterDays);
        filter_reset_days();
    }
}


static void filter_reset_scr_init(void)
{
    filter_reset_screen = lv_obj_create(NULL);

    static lv_obj_t *lable_reset_warning;
    lable_reset_warning = lv_label_create(filter_reset_screen);
    lv_label_set_text(lable_reset_warning,"Press OK to continue");
    lv_obj_set_style_text_font(lable_reset_warning, &lv_font_montserrat_18, 0);
    lv_obj_align(lable_reset_warning, LV_ALIGN_CENTER, 0, -50);


    btn_cancel = lv_btn_create(filter_reset_screen);
    lv_obj_set_size(btn_cancel, 60, 25);
    lv_obj_add_event_cb(btn_cancel, reset_event_hanlder , LV_EVENT_CLICKED, NULL);
    lv_obj_align_to(btn_cancel, lable_reset_warning,  LV_ALIGN_OUT_BOTTOM_LEFT, 0, 15);
    static lv_obj_t *label_cancel;
    label_cancel = lv_label_create(btn_cancel);
    lv_label_set_text(label_cancel, "Cancel");
    lv_obj_center(label_cancel);


    btn_OK = lv_btn_create(filter_reset_screen);
    //lv_obj_set_size(btn_OK, lv_obj_get_width(btn_cancel), lv_obj_get_height(btn_cancel));
    lv_obj_set_size(btn_OK, 60, 25);
    lv_obj_add_event_cb(btn_OK, reset_event_hanlder , LV_EVENT_CLICKED, NULL);
    lv_obj_align_to(btn_OK, lable_reset_warning,  LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 15);
    static lv_obj_t *label_OK;

    label_OK = lv_label_create(btn_OK);
    lv_label_set_text(label_OK, "OK");

    lv_obj_center(label_OK);

}





/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void lvgl_update_label_number(tdsData_t data)
{
    char numStr[20] = {0};
    switch(data.dataSource){
        case IN_TDS:
            sprintf(numStr, "#0000ff %d#", (int)data.value);
            lv_label_set_text(lable_incomming_tds, numStr);
            break;
        case OUT_TDS:
            sprintf(numStr, "#009688 %d#", (int)data.value);
            lv_label_set_text(lable_purifed_tds, numStr);
            break;
        case SERVE_DAYS:
            sprintf(numStr, "#FF9800 %d#", (int)data.value);
            lv_label_set_text(lable_days_num, numStr);
            break;
        default:
            break;
    }
}





void lvgl_tds_display_init(void){
    tds_screen = lv_obj_create(NULL);
    lv_scr_load(tds_screen);

    // incomming water lable ----------------------------------------
    static lv_obj_t *lable_incomming;
    lable_incomming = lv_label_create(tds_screen);
    lv_label_set_recolor(lable_incomming, true);  
    //lv_label_set_text(lable_incomming,"Source water TDS : #0000ff 450#");
    lv_label_set_text(lable_incomming,"Source water TDS : ");
    lv_obj_set_style_text_font(lable_incomming, &lv_font_montserrat_18, 0);
    lv_obj_align(lable_incomming, LV_ALIGN_TOP_LEFT, 5, 15);

    lable_incomming_tds = lv_label_create(tds_screen);
    lv_label_set_recolor(lable_incomming_tds, true);  
    lv_obj_set_style_text_font(lable_incomming_tds, &lv_font_montserrat_18, 0);
    lv_obj_align_to(lable_incomming_tds, lable_incomming, LV_ALIGN_OUT_RIGHT_MID, 5, 0);


    // purified water label -------------------------------------------
    static lv_obj_t *lable_purified;
    lable_purified = lv_label_create(tds_screen);
    lv_label_set_recolor(lable_purified, true);  
    //lv_label_set_text(lable_purified, "Purified water TDS : #009688 15#");
    lv_label_set_text(lable_purified, "Purified water TDS : ");
    lv_obj_set_style_text_font(lable_purified, &lv_font_montserrat_18, 0);
    lv_obj_align_to(lable_purified, lable_incomming, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);

    lable_purifed_tds = lv_label_create(tds_screen);
    lv_label_set_recolor(lable_purifed_tds, true);  
    lv_obj_set_style_text_font(lable_purifed_tds, &lv_font_montserrat_18, 0);
    lv_obj_align_to(lable_purifed_tds, lable_purified, LV_ALIGN_OUT_RIGHT_MID, 5, 0);


    // filter count days label -------------------------------------------
    static lv_obj_t *lable_days;
    lable_days = lv_label_create(tds_screen);
    lv_label_set_recolor(lable_days, true);
    lv_label_set_text(lable_days, "Filter service days : ");
    lv_obj_set_style_text_font(lable_days, &lv_font_montserrat_18, 0);
    lv_obj_align_to(lable_days, lable_purified, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);

    
    lable_days_num = lv_label_create(tds_screen);
    lv_label_set_recolor(lable_days_num, true);
    //lv_label_set_text(lable_days_num, "#FF9800 45#");
    lv_obj_set_style_text_font(lable_days_num, &lv_font_montserrat_18, 0);
    lv_obj_align_to(lable_days_num, lable_days, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    lv_obj_add_flag(lable_days_num,  LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lable_days_num, tds_filter_num_click_cb, LV_EVENT_CLICKED, NULL);
    


    static lv_obj_t *lable_reset;
    lable_reset = lv_label_create(tds_screen);
    lv_label_set_recolor(lable_reset, true);
    lv_label_set_text(lable_reset, "(Tap the #FF9800 DAY# to reset)");
    lv_obj_set_style_text_font(lable_reset, &lv_font_montserrat_18, 0);
    lv_obj_align_to(lable_reset, lable_days, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 3);

    static lv_obj_t *filter_gif;
    filter_gif = lv_gif_create(tds_screen);
    lv_gif_set_src(filter_gif, &filter_gif_160x160);
    lv_obj_align(filter_gif,LV_ALIGN_BOTTOM_MID, 0, 0);

    filter_reset_scr_init();
}
    
void lvgl_receive_tds_data(void)
{
    BaseType_t xStatus;
    tdsData_t receivedData;

    xStatus = xQueueReceive(tdsQueue, &receivedData, 0);
    if(xStatus == pdPASS)
    {
        switch (receivedData.dataSource)
        {
        case IN_TDS:
            
            break;
        case OUT_TDS:
            break;
        
        case SERVE_DAYS:
            break;
        
        default:
            break;
        }
    }
}