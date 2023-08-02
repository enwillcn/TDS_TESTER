#ifndef __TDS_DISPLAY_H__
#define __TDS_DISPLAY_H__

typedef enum {
    IN_TDS,
    OUT_TDS,
    SERVE_DAYS,
}dataSource_t;

typedef struct{
    dataSource_t dataSource;
    uint32_t          value; 
}tdsData_t;



void lvgl_tds_display_init(void);
void lvgl_update_label_number(tdsData_t data);



#endif /* __TDS_DISPLAY_H__ */