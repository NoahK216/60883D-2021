#pragma once
#include "include.cpp"
#define BAR_OFFSET 35
#define BAR_HEIGHT 200
#define BAR_WIDTH 30
#define BAR_RANGE_MIN 0
#define BAR_RANGE_MAX 100
#define ANIM_TIME 200
constexpr float VAL_LABEL_CONSTANT = BAR_HEIGHT/(BAR_RANGE_MAX - BAR_RANGE_MIN);

#define WARNING_THRESH 45
#define DANGER_THRESH 55

//Define the names of the motors
#define BAR_MOTOR_1 frontleft
#define BAR_MOTOR_2 midleft
#define BAR_MOTOR_3 backleft
#define BAR_MOTOR_4 frontright
#define BAR_MOTOR_5 midright
#define BAR_MOTOR_6 backright
#define BAR_MOTOR_7 lift
#define BAR_MOTOR_8 conveyormotor

lv_obj_t* parent = lv_obj_create(lv_scr_act(), NULL);

lv_obj_t* bar1Bar = lv_bar_create(parent, NULL);
lv_obj_t* bar2Bar = lv_bar_create(parent, NULL);
lv_obj_t* bar3Bar = lv_bar_create(parent, NULL);
lv_obj_t* bar4Bar = lv_bar_create(parent, NULL);
lv_obj_t* bar5Bar = lv_bar_create(parent, NULL);
lv_obj_t* bar6Bar = lv_bar_create(parent, NULL);
lv_obj_t* bar7Bar = lv_bar_create(parent, NULL);
lv_obj_t* bar8Bar = lv_bar_create(parent, NULL);

lv_obj_t* bar1Lable = lv_label_create(parent, NULL);
lv_obj_t* bar2Lable = lv_label_create(parent, NULL);
lv_obj_t* bar3Lable = lv_label_create(parent, NULL);
lv_obj_t* bar4Lable = lv_label_create(parent, NULL);
lv_obj_t* bar5Lable = lv_label_create(parent, NULL);
lv_obj_t* bar6Lable = lv_label_create(parent, NULL);
lv_obj_t* bar7Lable = lv_label_create(parent, NULL);
lv_obj_t* bar8Lable = lv_label_create(parent, NULL);

lv_obj_t* bar1Val = lv_label_create(parent, NULL);
lv_obj_t* bar2Val = lv_label_create(parent, NULL);
lv_obj_t* bar3Val = lv_label_create(parent, NULL);
lv_obj_t* bar4Val = lv_label_create(parent, NULL);
lv_obj_t* bar5Val = lv_label_create(parent, NULL);
lv_obj_t* bar6Val = lv_label_create(parent, NULL);
lv_obj_t* bar7Val = lv_label_create(parent, NULL);
lv_obj_t* bar8Val = lv_label_create(parent, NULL);

lv_style_t parentStyle;
lv_style_t barStyle;
lv_style_t barIndcDefaultStyle;
lv_style_t barIndcWarningStyle;
lv_style_t barIndcDangerStyle;
lv_style_t textStyle;


void setBarAttributes(lv_obj_t* barObj, lv_obj_t* allignTo, int offSet = BAR_OFFSET){
    lv_bar_set_range(barObj, BAR_RANGE_MIN, BAR_RANGE_MAX);
    lv_bar_set_style(barObj, LV_BAR_STYLE_BG, &barStyle);
    lv_bar_set_style(barObj, LV_BAR_STYLE_INDIC, &barIndcDefaultStyle);
    lv_obj_set_size(barObj, BAR_WIDTH, BAR_HEIGHT);
    lv_obj_align(barObj, allignTo, LV_ALIGN_IN_LEFT_MID, offSet, 0);
}


void setBarValAndColor(lv_obj_t* barObj, int val, lv_obj_t* labelObj){
    lv_bar_set_value_anim(barObj, val, ANIM_TIME);
    if(val >= DANGER_THRESH){lv_bar_set_style(barObj, LV_BAR_STYLE_INDIC, &barIndcDangerStyle);}
    else if(val >= WARNING_THRESH){lv_bar_set_style(barObj, LV_BAR_STYLE_INDIC, &barIndcWarningStyle);}
    else{lv_bar_set_style(barObj, LV_BAR_STYLE_INDIC, &barIndcDefaultStyle);}
    char buffer[5];
    sprintf(buffer, "%i", val);
    lv_label_set_text(labelObj, buffer);
    lv_obj_align(labelObj, barObj, LV_ALIGN_IN_BOTTOM_MID, 0, -(val*VAL_LABEL_CONSTANT));
}


void initBarGraph(){
    //INITIALIZE STYLES
    lv_style_copy(&parentStyle, &lv_style_plain);
    parentStyle.body.opa = 0;
    parentStyle.body.radius = 0;

    lv_style_copy(&barStyle, &lv_style_transp);

    lv_style_copy(&barIndcDefaultStyle, &lv_style_plain);
    barIndcDefaultStyle.body.main_color = LV_COLOR_MAKE(0, 255, 0);
    barIndcDefaultStyle.body.grad_color = LV_COLOR_MAKE(0, 255, 0);
    barIndcDefaultStyle.body.radius = 0;
    barIndcDefaultStyle.body.border.opa = LV_OPA_0;
    barIndcDefaultStyle.body.padding.hor = 0;            
    barIndcDefaultStyle.body.padding.ver = 0;

    lv_style_copy(&barIndcWarningStyle, &barIndcDefaultStyle);
    barIndcWarningStyle.body.main_color = LV_COLOR_MAKE(255, 255, 0);
    barIndcWarningStyle.body.grad_color = LV_COLOR_MAKE(255, 255, 0);

    lv_style_copy(&barIndcDangerStyle, &barIndcDefaultStyle);
    barIndcDangerStyle.body.main_color = LV_COLOR_MAKE(255, 0, 0);
    barIndcDangerStyle.body.grad_color = LV_COLOR_MAKE(255, 0, 0);

    lv_style_copy(&textStyle, &lv_style_plain);
    textStyle.text.color = LV_COLOR_WHITE;
    
    //SET THE PARENT OBJECT ATTRIBUTES

    lv_obj_set_style(parent, &parentStyle);
    lv_obj_set_pos(parent, 0, 0);
    lv_obj_set_size(parent, 340, 240);

    //SET THE BAR ATTRIBUTES

    setBarAttributes(bar1Bar, NULL, 10);
    setBarAttributes(bar2Bar, bar1Bar);
    setBarAttributes(bar3Bar, bar2Bar);
    setBarAttributes(bar4Bar, bar3Bar);
    setBarAttributes(bar5Bar, bar4Bar);
    setBarAttributes(bar6Bar, bar5Bar);
    setBarAttributes(bar7Bar, bar6Bar);
    setBarAttributes(bar8Bar, bar7Bar);

    //SET THE LABEL ATTRIBUTES

    lv_label_set_text(bar1Lable, "FL");
    lv_label_set_style(bar1Lable, &textStyle);
    lv_obj_align(bar1Lable, bar1Bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar2Lable, "ML");
    lv_label_set_style(bar2Lable, &textStyle);
    lv_obj_align(bar2Lable, bar2Bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar3Lable, "BL");
    lv_label_set_style(bar3Lable, &textStyle);
    lv_obj_align(bar3Lable, bar3Bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar4Lable, "FR");
    lv_label_set_style(bar4Lable, &textStyle);
    lv_obj_align(bar4Lable, bar4Bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar5Lable, "MR");
    lv_label_set_style(bar5Lable, &textStyle);
    lv_obj_align(bar5Lable, bar5Bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar6Lable, "BR");
    lv_label_set_style(bar6Lable, &textStyle);
    lv_obj_align(bar6Lable, bar6Bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar7Lable, "L");
    lv_label_set_style(bar7Lable, &textStyle);
    lv_obj_align(bar7Lable, bar7Bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar8Lable, "C");
    lv_label_set_style(bar8Lable, &textStyle);
    lv_obj_align(bar8Lable, bar8Bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    //SET STYLE FOR VAL LABELS
    lv_label_set_style(bar1Val, &textStyle);
    lv_label_set_style(bar2Val, &textStyle);
    lv_label_set_style(bar3Val, &textStyle);
    lv_label_set_style(bar4Val, &textStyle);
    lv_label_set_style(bar5Val, &textStyle);
    lv_label_set_style(bar6Val, &textStyle);
    lv_label_set_style(bar7Val, &textStyle);
    lv_label_set_style(bar8Val, &textStyle);


    setBarValAndColor(bar1Bar, 35, bar1Val);
    setBarValAndColor(bar2Bar, 35, bar2Val);
    setBarValAndColor(bar3Bar, 35, bar3Val);
    setBarValAndColor(bar4Bar, 35, bar4Val);
    setBarValAndColor(bar5Bar, 35, bar5Val);
    setBarValAndColor(bar6Bar, 35, bar6Val);
    setBarValAndColor(bar7Bar, 35, bar7Val);
    setBarValAndColor(bar8Bar, 35, bar8Val);
}

void updateBarGraph(){
    setBarValAndColor(bar1Bar, BAR_MOTOR_1.get_temperature(), bar1Val);
    setBarValAndColor(bar2Bar, BAR_MOTOR_2.get_temperature(), bar2Val);
    setBarValAndColor(bar3Bar, BAR_MOTOR_3.get_temperature(), bar3Val);
    setBarValAndColor(bar4Bar, BAR_MOTOR_4.get_temperature(), bar4Val);
    setBarValAndColor(bar5Bar, BAR_MOTOR_5.get_temperature(), bar5Val);
    setBarValAndColor(bar6Bar, BAR_MOTOR_6.get_temperature(), bar6Val);
    setBarValAndColor(bar7Bar, BAR_MOTOR_7.get_temperature(), bar7Val);
    setBarValAndColor(bar8Bar, BAR_MOTOR_8.get_temperature(), bar8Val);
}

void updateBarGraph_fn(void* param){
    uint32_t startTime = pros::millis();
    pros::Task::delay_until(&startTime, 1000);
    while(true){
        setBarValAndColor(bar1Bar, BAR_MOTOR_1.get_temperature(), bar1Val);
        setBarValAndColor(bar2Bar, BAR_MOTOR_2.get_temperature(), bar2Val);
        setBarValAndColor(bar3Bar, BAR_MOTOR_3.get_temperature(), bar3Val);
        setBarValAndColor(bar4Bar, BAR_MOTOR_4.get_temperature(), bar4Val);
        setBarValAndColor(bar5Bar, BAR_MOTOR_5.get_temperature(), bar5Val);
        setBarValAndColor(bar6Bar, BAR_MOTOR_6.get_temperature(), bar6Val);
        setBarValAndColor(bar7Bar, BAR_MOTOR_7.get_temperature(), bar7Val);
        setBarValAndColor(bar8Bar, BAR_MOTOR_8.get_temperature(), bar8Val);
        pros::Task::delay_until(&startTime, 1000);  
    }
}