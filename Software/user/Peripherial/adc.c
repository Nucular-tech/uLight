/*
 * adc.c
 *
 *  Created on: 16 feb. 2020 ã.
 *      Author: VasiliSk
 */
#include "Resconfig.h"
#include "adc.h"
#include "adc_hal.h"
#include "Data/hwconfig.h"
#include "Data/parameters.h"

typedef struct {
	int16_t Amp;
	int16_t V12;
	int16_t Tint;
	int16_t T1;
	int16_t T2;
	int16_t VBrake;
	int16_t VThrottle;
} ADC1_SmpRaw_t;

enum {
	ADC_Ch_Amp, ADC_Ch_V12, ADC_Ch_Tint, ADC_Ch_T2 = 5, ADC_Ch_T1 = 7, ADC_Ch_VBrake, ADC_Ch_VThrottle,
};

#define ADCs_SamplingTime1   (ADC_SMPR1(SMPR_71CYCLES_5, CHNL_10)\
							| ADC_SMPR1(SMPR_71CYCLES_5, CHNL_11)\
                            | ADC_SMPR1(SMPR_71CYCLES_5, CHNL_12)\
                            | ADC_SMPR1(SMPR_71CYCLES_5, CHNL_13))

#define ADCs_SamplingTime2   (ADC_SMPR2(SMPR_71CYCLES_5, CHNL_1)\
                            | ADC_SMPR2(SMPR_71CYCLES_5, CHNL_2)\
                            | ADC_SMPR2(SMPR_71CYCLES_5, CHNL_3)\
                            | ADC_SMPR2(SMPR_71CYCLES_5, CHNL_4)\
                            | ADC_SMPR2(SMPR_71CYCLES_5, CHNL_5)\
                            | ADC_SMPR2(SMPR_71CYCLES_5, CHNL_6)\
                            | ADC_SMPR2(SMPR_71CYCLES_5, CHNL_7))
//private functions
uint16_t adc_calibrate(ADC_TypeDef *adc);
int32_t calc_temp(int32_t raw, Tsensor_t type);

//private variables
volatile ADC1_SmpRaw_t adc1_samples;

//public variables
volatile ADC1_Smp_t ADC_Values;
volatile ADC1_Smp_t ADC_ValuesF;

void ADC_user_init(void) {
	// ADC 1: External trigger SWSTART, continious with dma,ExtSel=Tim3 TRGO, JExt
	// trigger Timer1 TRGO event
	ADC1->SR = 0;
	ADC1->CR2 = ADC_CR2_EXTSEL_2 | ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL | ADC_CR2_DMA;
	ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE;
	ADC1->SMPR1 = ADCs_SamplingTime1;
	ADC1->SMPR2 = ADCs_SamplingTime2;
	ADC1->SQR1 = ADC_SQR1_SetNumberOfConv(7);	// 7 channels
	ADC1->SQR2 = ADC_SQR2_RK(ADC_Ch_VThrottle, 7);
	ADC1->SQR3 = ADC_SQR3_RK(ADC_Ch_Amp, 1) //
	| ADC_SQR3_RK(ADC_Ch_V12, 2) | ADC_SQR3_RK(ADC_Ch_Tint, 3) | ADC_SQR3_RK(ADC_Ch_T1, 4) | ADC_SQR3_RK(ADC_Ch_T2, 5) | ADC_SQR3_RK(ADC_Ch_VBrake, 6);
	ADC1->JSQR = 0; // injected: pha for example, 1 conversion

	// ADC 2: fet temp on software start and injected, master ADC1, here JSWSTART
	/*ADC2->SR = 0;			// slave swstrt trigger
	 ADC2->CR2 = ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG | ADC_CR2_JEXTTRIG | ADC_CR2_JEXTSEL;
	 ADC2->CR1 = ADC_CR1_SCAN;
	 ADC2->SMPR1 = ADCs_SamplingTime1;
	 ADC2->SMPR2 = ADCs_SamplingTime2;
	 ADC3->SQR1 = ADC_SQR1_SetNumberOfConv(1);	// 1channels
	 ADC2->SQR3 = ADC_SQR3_RK(CHNL_FetTemp, 1);
	 ADC2->JSQR = CHNL_JPHB;*/

	adc_calibrate(ADC1);
	//ADC_Calibrate(ADC2);

	// DUALMOD injected simultaneous mode
	//ADC1->CR1 |= (uint32_t) ADC_CR1_MODE_INJECSIMULT;

	// ADC 1&2 DMA circular buffer, memory increment, perepherial and memory 16b,
	// transfer complet interrupts ON
	DMA1_Channel1->CCR = DMA_CCR1_CIRC | DMA_CCR1_MINC | DMA_CCR1_PSIZE_0 | DMA_CCR1_MSIZE_0 | DMA_CCR1_TCIE;
	DMA1_Channel1->CPAR = (uint32_t) & ADC1->DR;
	DMA1_Channel1->CMAR = (uint32_t) &adc1_samples;
	DMA1_Channel1->CNDTR = 7;

	DMA1_Channel1->CCR |= DMA_CCR1_EN;

	NVIC_SetPriority(DMA1_Channel1_IRQn, 4);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	// ADC 1-3 IRQ
	//NVIC_SetPriority(ADC1_2_IRQn, 3);

}

uint16_t adc_calibrate(ADC_TypeDef *adc) {
	/* 1. Calibration prerequisite:                                             */
	/*    - ADC must be disabled for at least two ADC clock cycles in disable   */
	/*      mode before ADC enable                                              */
	/* Stop potential conversion on going, on regular and injected groups       */
	/* Disable ADC peripheral */
	adc->CR2 &= ~ADC_CR2_ADON;
	/* Hardware prerequisite: delay before starting the calibration.          */
	/*  - Computation of CPU clock cycles corresponding to ADC clock cycles.  */
	/*  - Wait for the expected ADC clock cycles delay */
	uint16_t wait_loop_index = (SystemCoreClock / ADC_CLOCK * 2);
	while (wait_loop_index != 0)
		wait_loop_index--;
	/* 2. Enable the ADC peripheral */
	adc->CR2 |= ADC_CR2_ADON;
	/* 3. Resets ADC calibration registers */
	adc->CR2 |= ADC_CR2_RSTCAL;
	wait_loop_index = (SystemCoreClock / 1000 * ADC_CALIBRATION_TIMEOUT) / 2;
	/* Wait for calibration reset completion */
	while (adc->CR2 & ADC_CR2_RSTCAL) {
		wait_loop_index--;
		if (wait_loop_index == 0)
			return 1;
	}
	/* 4. Start ADC calibration */
	adc->CR2 |= ADC_CR2_CAL;

	wait_loop_index = (SystemCoreClock / 1000 * ADC_CALIBRATION_TIMEOUT) / 2;
	/* Wait for calibration completion */
	while (adc->CR2 & ADC_CR2_CAL) {
		wait_loop_index--;
		if (wait_loop_index == 0)
			return 1;
	}
	return 0;
}

void DMA1_Channel1_IRQHandler(void) {
	// global interrupt reset
	DMA1->IFCR = DMA_IFCR_CGIF1;

	ADC_Values.Amp = ((int32_t) adc1_samples.Amp * HWConfig.ADC.Amp_mult) / HWConfig.ADC.Amp_div;
	ADC_Values.V12 = ((int32_t) adc1_samples.V12 * HWConfig.ADC.V12_mult) / HWConfig.ADC.V12_div;
	ADC_Values.VBrake = ((int32_t) adc1_samples.VBrake * HWConfig.ADC.VthVbrk_mult) / HWConfig.ADC.VthVbrk_div;
	ADC_Values.VThrottle = ((int32_t) adc1_samples.VThrottle * HWConfig.ADC.VthVbrk_mult) / HWConfig.ADC.VthVbrk_div;
	ADC_Values.Tint = calc_temp(adc1_samples.Tint, Tsensor_NTC10K3380);
	ADC_Values.T1 = calc_temp(adc1_samples.T1, Config.InputsCfg.T1);
	ADC_Values.T2 = calc_temp(adc1_samples.T2, Config.InputsCfg.T2);

	int32_t *adc_filtered = (void*) &ADC_ValuesF;
	int32_t *adc_value = (void*) &ADC_Values;
	for (uint i = 0; i < sizeof(ADC_Values) / 4; i++) {
		*adc_filtered = Filter_LPibetter(*adc_value, *adc_filtered, 7, 110);
		adc_filtered++;
		adc_value++;
	}
}

int32_t Filter_LPi(int32_t new, int32_t previous, uint8_t beta) {
	int64_t buffer = ((int64_t) previous << beta) - previous;
	buffer += new;
	return (int32_t) (buffer >>= beta);
}

int32_t Filter_LPibetter(int32_t new, int32_t previous, uint8_t beta, int32_t K) {
	int32_t Z = K * (previous - new);
	if (Z < 0) {
		Z = Z + ((1 << beta) - 1);
	}
	int32_t y = new + (Z >> beta);
	return y;
}

int32_t calc_temp(int32_t raw, Tsensor_t type) {
	int32_t temp;
	int32_t pow2 = raw * raw;

	switch (type) {
	default:
		temp = 234;
		break;
	case Tsensor_NTC10K3380: {
		if (raw > 2500) {
			temp = (((int32_t) (((((int64_t) pow2 * raw) >> 16) * -14430708) >> 16) + (int32_t) (((int64_t) pow2 * 1696358) >> 16) + (raw * -82720) + 106399799) >> 16);
		} else if (raw > 1000) {
			temp = (((int32_t) (((((int64_t) pow2 * raw) >> 16) * -12296842) >> 16) + (int32_t) (((int64_t) pow2 * 1209989) >> 16) + (raw * -56206) + 78684933) >> 16);
		} else {
			temp = (((int32_t) (((((int64_t) pow2 * raw) >> 16) * -223039414) >> 16) + (int32_t) (((int64_t) pow2 * 9322894) >> 16) + (raw * -162054) + 109513152) >> 16);
		}
	}
		break;
	case Tsensor_NTC10K3950: {
		if (raw > 2300) {
			temp = (((int32_t) (((((int64_t) pow2 * raw) >> 16) * -17547019) >> 16) + (int32_t) (((int64_t) pow2 * 2118824) >> 16) + (raw * -99297) + 119879668) >> 16);
		} else if (raw > 750) {
			temp = (((int32_t) (((((int64_t) pow2 * raw) >> 16) * -11466230) >> 16) + (int32_t) (((int64_t) pow2 * 1100240) >> 16) + (raw * -49255) + 69839119) >> 16);
		} else {
			temp = (((int32_t) (((((int64_t) pow2 * raw) >> 16) * -369412926) >> 16) + (int32_t) (((int64_t) pow2 * 12457346) >> 16) + (raw * -171761) + 99558307) >> 16);
		}
	}
		break;
	case Tsensor_KTY84_130: {
		temp = (((int32_t) (((((int64_t) pow2 * raw) >> 16) * 1217890196) >> 16) + (int32_t) (((int64_t) pow2 * -36478711) >> 16) + (raw * 619092) + -101962104) >> 16);
	}
		break;
	}
	return temp;
}

