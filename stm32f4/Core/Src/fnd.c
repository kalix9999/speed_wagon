/*
 * fnd.c
 *
 *  Created on: Dec 19, 2025
 *      Author: TaeGon_laptop
 */


#include "fnd.h"
#include "main.h"


// 1. 세그먼트 핀 (A, B, C, D, E, F, G, DP 순서)
// 구조체를 사용하여 포트와 핀을 묶어서 관리 (정석적인 방법)
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} FND_PinConfig;

const FND_PinConfig SEG_PINS[8] = {
    {seg_11_GPIO_Port, seg_11_Pin}, // A
    {seg_7_GPIO_Port, seg_7_Pin}, // B
    {seg_4_GPIO_Port, seg_4_Pin}, // C
    {seg_2_GPIO_Port, seg_2_Pin}, // D
    {seg_1_GPIO_Port, seg_1_Pin}, // E
    {seg_10_GPIO_Port, seg_10_Pin}, // F
    {seg_5_GPIO_Port, seg_5_Pin}, // G
    {seg_3_GPIO_Port, seg_3_Pin}  // DP
};

// 2. 자릿수 선택 핀 (Digit 1, 2, 3, 4 순서)
const FND_PinConfig DIGIT_PINS[4] = {
    {seg_12_GPIO_Port, seg_12_Pin}, // D1 (천의 자리)
    {seg_9_GPIO_Port, seg_9_Pin}, // D2 (백의 자리)
    {seg_8_GPIO_Port, seg_8_Pin}, // D3 (십의 자리)
    {seg_6_GPIO_Port, seg_6_Pin}  // D4 (일의 자리)
};

// ==========================================
// [내부 변수 및 패턴]
// ==========================================

// 표시할 숫자 저장 (내부 전역 변수)
static volatile uint32_t g_fndDisplayValue = 0;

// 숫자 패턴 (0~9) - 1이 켜짐으로 정의 (출력 시 반전 처리)
static const uint8_t NUMBER_PATTERN[11] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x00  // 10 (전부 꺼짐)
};

// ==========================================
// [함수 구현]
// ==========================================
void FND_OFF(void){
	for(int i=0; i<4; i++) {
		HAL_GPIO_WritePin(DIGIT_PINS[i].port, DIGIT_PINS[i].pin, GPIO_PIN_RESET);
	}
}

void FND_Init(void) {
    // GPIO 초기화는 보통 main.c의 MX_GPIO_Init에서 수행하므로
    // 여기서는 변수 초기화 정도만 수행합니다.
    g_fndDisplayValue = 0;
    FND_OFF();
}

void FND_SetNumber(uint32_t number) {
    if(number > 9999) number = 9999;
    if(number < 0) number = 0;
    g_fndDisplayValue = number;
}

void FND_Update(void) {
    static int current_digit = 0; // 현재 켜야 할 자리 (0~3)

    // 1. 잔상 제거를 위해 모든 자릿수 끄기 (Common Anode -> LOW)
    FND_OFF();

    // 2. 현재 자리에 표시할 숫자 계산
    int num_to_show = 0;
    if(current_digit == 0)      num_to_show = (g_fndDisplayValue >= 1000) ? (g_fndDisplayValue / 1000) % 10 : 10;
    else if(current_digit == 1) num_to_show = (g_fndDisplayValue >= 100) ? (g_fndDisplayValue / 100) % 10 : 10;
    else if(current_digit == 2) num_to_show = (g_fndDisplayValue >= 10) ? (g_fndDisplayValue / 10) % 10 : 0;
    else if(current_digit == 3) num_to_show = g_fndDisplayValue % 10;

    // 3. 세그먼트 제어 (Common Anode -> 켜려면 LOW)
    uint8_t pattern = NUMBER_PATTERN[num_to_show];
    if(current_digit==2) pattern |= 0x80; // 10이하면 소수점(dp) 켜기
    for(int i=0; i<8; i++) {
        if((pattern >> i) & 1) {
            HAL_GPIO_WritePin(SEG_PINS[i].port, SEG_PINS[i].pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(SEG_PINS[i].port, SEG_PINS[i].pin, GPIO_PIN_SET);
        }
    }

    // 4. 해당 자릿수 켜기 (Common Anode -> 켜려면 HIGH)
    HAL_GPIO_WritePin(DIGIT_PINS[current_digit].port, DIGIT_PINS[current_digit].pin, GPIO_PIN_SET);

    // 5. 다음 자릿수 준비
    current_digit = (current_digit+1) % 4;
}
