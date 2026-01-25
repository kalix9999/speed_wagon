/*
 * menu.h
 *
 *  Created on: Jan 2, 2026
 *      Author: 1-06
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include <stdint.h>
#include "arm_math.h"

typedef enum {
    STATE_DASHBOARD,  // 현재 속도 표시 화면
    STATE_MENU_LIST,   // 메뉴 항목 선택 화면
    STATE_SET_VALUE    // 특정 환경변수 값 수정 화면
} UI_State;

// 메뉴 항목 구조체 정의
typedef struct {
    char* title;             // 메뉴 이름
    volatile uint32_t* target_value;  // 변경할 변수의 주소
    int min;                 // 설정 최솟값
    int max;                 // 설정 최댓값
    int step;				 // 값 조정시 간격
} MenuItem;

extern MenuItem menuItems[];

extern volatile uint32_t debug_speed;   // 계산된 속도
extern volatile uint32_t debug_speed_x10; // 계산된 속도의 10배
extern volatile uint32_t debug_freq;    // 계산된 주파수
extern volatile int32_t debug_maxVal;   // 신호 세기 (Magnitude)
extern volatile uint32_t debug_isr_cnt; // 인터럽트 횟수 카운터
extern volatile q15_t debug_mag; // 푸리에 편환 주파수별 세기 그래프 보기용도

extern volatile uint32_t TH_OVERSPEED_km_h;
extern volatile uint32_t TH_NOISE;
extern volatile int32_t debug_maxVal;
extern volatile uint32_t noise_search_trigger; // 0: 대기, 1: 측정중, 2: 완료
extern uint8_t noise_bin_mask[]; // 0: 정상, 1: 노이즈 주파수로 마스킹됨

extern int MENU_COUNT;

void menu_init();
void UpdateLCD(UI_State state, int menuIdx);

#endif /* INC_MENU_H_ */
