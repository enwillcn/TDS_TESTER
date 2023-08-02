    /*Create a scale for the minutes*/
    /*61 ticks in a 360 degrees range (the last and the first line overlaps)*/
    lv_meter_scale_t * scale_min = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale_min, 61, 1, 10, lv_palette_main(LV_PALETTE_RED));
    lv_meter_set_scale_range(meter, scale_min, 0, 60, 360, 270);

//    /*Create an other scale for the hours. It's only visual and contains only major ticks*/
    lv_meter_scale_t * scale_hour = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale_hour, 12, 0, 0, lv_palette_main(LV_PALETTE_GREY));               /*12 ticks*/
    lv_meter_set_scale_major_ticks(meter, scale_hour, 1, 2, 20, lv_color_black(), 10);    /*Every tick is major*/
    lv_meter_set_scale_range(meter, scale_hour, 1, 12, 330, 300);       /*[1..12] values in an almost full circle*/

    lv_meter_scale_t * scale_360 = lv_meter_add_scale(meter);
    //lv_meter_set_scale_ticks(meter, scale_360, 240, 0, 0, lv_palette_main(LV_PALETTE_YELLOW));               /*12 ticks*/
    //lv_meter_set_scale_major_ticks(meter, scale_360, 1, 2, 20, lv_color_black(), 10);    /*Every tick is major*/
    lv_meter_set_scale_range(meter, scale_360, 0, 360, 360, 270);       /*[1..12] values in an almost full circle*/


    LV_IMG_DECLARE(img_hand)

    /*Add a the hands from images*/
//    lv_meter_indicator_t * indic_min = lv_meter_add_needle_img(meter, scale_min, &img_hand, 5, 5);
//    lv_meter_indicator_t * indic_hour = lv_meter_add_needle_img(meter, scale_360, &img_hand, 5, 5);

    lv_meter_indicator_t * indic_sec = lv_meter_add_needle_line(meter, scale_min, 2 ,lv_palette_main(LV_PALETTE_BLUE), 5);
    lv_meter_indicator_t * indic_min = lv_meter_add_needle_line(meter, scale_360, 3 ,lv_color_black(), -5);
    lv_meter_indicator_t * indic_hour = lv_meter_add_needle_line(meter, scale_360, 4 ,lv_color_black(), -15);

    lv_meter_set_indicator_end_value(meter, indic_sec, 0);

    /*Create an animation to set the value*/
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_exec_cb(&a, set_value);
    lv_anim_set_values(&a, 0, 60);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_time(&a, 60*1000);     /*2 sec for 1 turn of the minute hand (1 hour)*/
    lv_anim_set_var(&a, indic_sec);
    lv_anim_start(&a);

    lv_anim_set_var(&a, indic_hour);
    lv_anim_set_time(&a, 12*60*60*1000);    /*24 sec for 1 turn of the hour hand*/
    lv_anim_set_values(&a, 0, 360);
    lv_anim_start(&a);

    lv_anim_set_var(&a, indic_min);
    lv_anim_set_time(&a, 60*60*1000);    /*24 sec for 1 turn of the hour hand*/
    lv_anim_set_values(&a, 0, 360);
    lv_anim_start(&a);