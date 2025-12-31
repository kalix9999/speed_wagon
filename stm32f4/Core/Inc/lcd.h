/*
 * lcd.h
 *
 *  Created on: Dec 31, 2025
 *      Author: 1-06
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f4xx_hal.h"

void lcd_init(void);   // LCD 초기화
void lcd_send_cmd(char cmd);  // 명령어 전송
void lcd_send_data(char data); // 데이터 전송
void lcd_send_string(char *str); // 문자열 전송
void lcd_put_cur(int row, int col); // 커서 이동
void lcd_clear(void); // 화면 지우기

#endif /* INC_LCD_H_ */
