/*
 * menu.c
 *
 *  Created on: Jan 2, 2026
 *      Author: 1-06
 */
#include "menu.h"
#include "lcd.h"
#include <stdio.h>

int MENU_COUNT;

MenuItem menuItems[] = {
    {"1.TH OverSpeed", &TH_OVERSPEED_km_h, 5, 100, 5} // 과속 기준값 설정
    ,{"2.TH_NOISE", &TH_NOISE, 0, 2000, 100} // 노이즈 기준치 값
};

void menu_init(){
	MENU_COUNT = sizeof(menuItems) / sizeof(MenuItem);
}

void UpdateLCD(UI_State state, int menuIdx) {
    lcd_clear();
    char buf[20];

    switch (state) {
        case STATE_DASHBOARD:
            lcd_put_cur(0, 0);
            lcd_send_string("Current Speed");
            sprintf(buf, "%lu.%lu km/h", debug_speed_x10/10, debug_speed_x10%10); // FFTTask에서 계산된 속도
            lcd_put_cur(1, 0);
            lcd_send_string(buf);
            break;

        case STATE_MENU_LIST:
            lcd_put_cur(0, 0);
            lcd_send_string("> Menu List");
            lcd_put_cur(1, 0);
            lcd_send_string(menuItems[menuIdx].title);
            break;

        case STATE_SET_VALUE:
            lcd_put_cur(0, 0);
            lcd_send_string(menuItems[menuIdx].title);
            sprintf(buf, "Value: %ld", *(menuItems[menuIdx].target_value));
            lcd_put_cur(1, 0);
            lcd_send_string(buf);
            break;
    }
}
