/*
 * pwm.h
 *
 *  Created on: 18 feb 2020 �.
 *      Author: VasiliSk
 */

#pragma once

#define TIM_MODE_UPCOUNTER				(0)
#define TIM_MODE_DOWNCOUNTER			(TIM_CR1_DIR)
#define TIM_MODE_CENTER_IFDOWN			(TIM_CR1_CMS_0)
#define TIM_MODE_CENTER_IFUP			(TIM_CR1_CMS_1)
#define TIM_MODE_CENTER_IFBOTH			(TIM_CR1_CMS)

#define TIM_OC1_ENABLE					(TIM_CCER_CC1E | TIM_CCER_CC1NE)
#define TIM_OC2_ENABLE					(TIM_CCER_CC2E | TIM_CCER_CC2NE)
#define TIM_OC3_ENABLE					(TIM_CCER_CC3E | TIM_CCER_CC3NE)
#define TIM_OC4_ENABLE					(TIM_CCER_CC4E)

#define TIM_OC1_FROZEN					((uint32_t)0x0000)
#define TIM_OC1_ACTIVE					(TIM_CCMR1_OC1M_0)
#define TIM_OC1_INACTIVE				(TIM_CCMR1_OC1M_1)
#define TIM_OC1_TOGGLE					(TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1)
#define TIM_OC1_PWM1					(TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)
#define TIM_OC1_PWM2					(TIM_CCMR1_OC1M)
#define TIM_OC1_FORCE_ACTIVE			(TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2)
#define TIM_OC1_FORCE_INACTIVE			(TIM_CCMR1_OC1M_2)

#define TIM_OC2_FROZEN					((uint32_t)0x0000)
#define TIM_OC2_ACTIVE					(TIM_OC1_ACTIVE << 8)
#define TIM_OC2_INACTIVE				(TIM_OC1_INACTIVE << 8)
#define TIM_OC2_TOGGLE					(TIM_OC1_TOGGLE << 8)
#define TIM_OC2_PWM1					(TIM_OC1_PWM1 << 8)
#define TIM_OC2_PWM2					(TIM_OC1_PWM2 << 8)
#define TIM_OC2_FORCE_ACTIVE			(TIM_OC1_FORCE_ACTIVE << 8)
#define TIM_OC2_FORCE_INACTIVE			(TIM_OC1_FORCE_INACTIVE << 8)

#define TIM_OC3_FROZEN					((uint32_t)0x0000)
#define TIM_OC3_ACTIVE					(TIM_OC1_ACTIVE)
#define TIM_OC3_INACTIVE				(TIM_OC1_INACTIVE)
#define TIM_OC3_TOGGLE					(TIM_OC1_TOGGLE)
#define TIM_OC3_PWM1					(TIM_OC1_PWM1)
#define TIM_OC3_PWM2					(TIM_OC1_PWM2)
#define TIM_OC3_FORCE_ACTIVE			(TIM_OC1_FORCE_ACTIVE)
#define TIM_OC3_FORCE_INACTIVE			(TIM_OC1_FORCE_INACTIVE)

#define TIM_OC4_PWM2					(TIM_OC1_PWM2 << 8)
#define TIM_OC4_PWM1					(TIM_OC1_PWM1 << 8)

#define TIM_CCMR1_PRELOAD_EN			(TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE)
#define TIM_CCMR2_PRELOAD_EN			(TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE)

enum {
	PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, IOPWM1, IOPWM2, IOPWM3, IOPWM4,
};

void PWMoutInit(void);
void PWMsetOutput(uint8_t pwm, uint16_t value);
void PWMDisableAll(void);
