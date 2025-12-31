/*
 * lcd.c
 *
 *  Created on: Dec 31, 2025
 *      Author: 1-06
 */


#include "lcd.h"

extern I2C_HandleTypeDef hi2c3;  // main.c에 정의된 i2c 핸들러 참조

#define SLAVE_ADDRESS_LCD 0x4E // 0x27 주소를 8비트로 변환 (0x27 << 1)

/**
 * @brief LCD 화면을 초기화합니다.
 */
void lcd_init(void) {
    // 4비트 초기화 시퀀스
    HAL_Delay(50);
    lcd_send_cmd(0x30);
    HAL_Delay(5);
    lcd_send_cmd(0x30);
    HAL_Delay(1);
    lcd_send_cmd(0x30);
    HAL_Delay(10);
    lcd_send_cmd(0x20);  // 4비트 모드 설정
    HAL_Delay(10);

    // 디스플레이 설정
    lcd_send_cmd(0x28); // 2라인, 5x8 Matrix
    HAL_Delay(1);
    lcd_send_cmd(0x08); // Display OFF
    HAL_Delay(1);
    lcd_send_cmd(0x01); // Clear Display
    HAL_Delay(2);
    lcd_send_cmd(0x06); // Entry mode
    HAL_Delay(1);
    lcd_send_cmd(0x0C); // Display ON, Cursor OFF
}

/**
 * @brief LCD에 명령어를 전송합니다.
 */
void lcd_send_cmd(char cmd) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);

    data_t[0] = data_u | 0x0C;  // en=1, rs=0 (명령어 모드, 백라이트 ON)
    data_t[1] = data_u | 0x08;  // en=0, rs=0
    data_t[2] = data_l | 0x0C;  // en=1, rs=0
    data_t[3] = data_l | 0x08;  // en=0, rs=0

    HAL_I2C_Master_Transmit(&hi2c3, SLAVE_ADDRESS_LCD, (uint8_t *)data_t, 4, 100);
}

/**
 * @brief LCD에 데이터를 전송합니다.
 */
void lcd_send_data(char data) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);

    data_t[0] = data_u | 0x0D;  // en=1, rs=1 (데이터 모드, 백라이트 ON)
    data_t[1] = data_u | 0x09;  // en=0, rs=1
    data_t[2] = data_l | 0x0D;  // en=1, rs=1
    data_t[3] = data_l | 0x09;  // en=0, rs=1

    HAL_I2C_Master_Transmit(&hi2c3, SLAVE_ADDRESS_LCD, (uint8_t *)data_t, 4, 100);
}



/**
 * @brief 특정 위치로 커서를 이동합니다.
 * @param row 행 (0 또는 1)
 * @param col 열 (0 ~ 15)
 */
void lcd_put_cur(int row, int col) {
    switch (row) {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd(col);
}

/**
 * @brief LCD 화면을 모두 지웁니다.
 */
void lcd_clear(void) {
    lcd_send_cmd(0x01);
    HAL_Delay(2);
}

/**
 * @brief 문자열을 출력합니다.
 */
void lcd_send_string(char *str) {
    while (*str) lcd_send_data(*str++);
}
