/*
 * fnd.h
 * 세븐세그먼트 제어용
 *  Created on: Dec 19, 2025
 *      Author: TaeGon_laptop
 */

#ifndef INC_FND_H_
#define INC_FND_H_
#include <stdint.h>

void FND_Init(void);

// FND 전부 OFF
void FND_OFF(void);

// 표시할 숫자 설정 (0 ~ 9999)
void FND_SetNumber(uint32_t number);

// FND 화면 갱신 (while문이나 타이머에서 계속 호출)
void FND_Update(void);

#endif /* INC_FND_H_ */
