#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f10x.h"
#include "armutils.h"
#include "Resconfig.h"
#include "network.h"
#include "levcan_events.h"
#include "Data/parameters.h"
#include "Data/hwconfig.h"
#include "storage.h"
#include "Peripherial/adc.h"
#include "logic.h"
#include "pwm.h"

#define CHECK_DWORD		0x20000000
#define PROGRAM_KEY		0x02CACEDA
#define SHUTDOWN_KEY	0xDEADBEEF

#ifdef DEBUG
#define sDEBUG(string) string" debug"
#else
#define sDEBUG(string) string
#endif

const Version_t VersionControl = { __DATE__, sDEBUG("0.6.4") };

typedef struct {
	uint32_t GoToBoot;
	uint8_t NodeID;
	uint8_t ServerID;
} BootCommand_t;
#define BootCommand ((BootCommand_t*)CHECK_DWORD)

// Private functions
void RCC_user_init(void);
void GPIO_user_init(void);
void GPIO_user_deinit(void);
void EXTI_init(void);
void GoToBoot(uint8_t bootloader, int8_t node_id, int8_t server_id);
void updateButtons(void);
void delay_ms(uint32_t delay);

//private variables
volatile uint32_t SystemTick = 0;
volatile uint32_t ADC_CLOCK;

int main(int argc, char *argv[]) {
	(void) argv;
	(void) argc;
	/*
	 GD32
	 CPUID 412FC231 DEVID 410
	 Cortex M3 r2p1

	 STM32
	 CPUID 411FC231 DEVID 410
	 Cortex M3 r1p1
	 */

	ARM_CheckAll();
	HWInit();
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	LoadDefaultParameters();
	Storage_Init();
	RCC_user_init();
	//port settings loaded here
	for (uint8_t i = 0; i < Save_number; i++) {
		if (Storage_LoadData(i) == ERROR) {
			Storage_SaveData(i); //length mismatch
		}
	}

	PWMoutInit();
	GPIO_user_init();
	ADC_user_init();
	EXTI_init();
	Network_Init(50 + STM_UID.UID_0 % 64);

	SysTick_Config(SystemCoreClock / 1000);
	NVIC_SetPriority(SysTick_IRQn, 1); //almost highest

	while (1) {

	}
}

void updateButtons(void) {
	RD.Buttons.OffBt = 0;
	RD.Buttons.OnBt = 1;
	RD.Buttons.Int1 = !GPIO_PIN_GET(P_IN1_Pin, P_IN1_GPIO_Port);
	RD.Buttons.Int2 = !GPIO_PIN_GET(P_IN2_Pin, P_IN2_GPIO_Port);
	RD.Buttons.Int3 = !GPIO_PIN_GET(P_IN3_Pin, P_IN3_GPIO_Port);
	RD.Buttons.Int4 = !GPIO_PIN_GET(P_IN4_Pin, P_IN4_GPIO_Port);
	RD.Buttons.Int5 = !GPIO_PIN_GET(P_IN5_Pin, P_IN5_GPIO_Port);
	RD.Buttons.Int6 = !GPIO_PIN_GET(P_IN6_Pin, P_IN6_GPIO_Port);

	//virtual input for t-sensors
	//get value and invert if needed. with 1C hysteresis
	uint8_t in7_t1_bool = ADC_ValuesF.T1 > ((int32_t) Config.InputsCfg.T1_Threshold * 10 - RD.Buttons.Int7_T1 * 10);
	in7_t1_bool = in7_t1_bool ^ Config.InputsCfg.T1_ThrMode;
	RD.Buttons.Int7_T1 = in7_t1_bool;

	uint8_t in8_t2_bool = ADC_ValuesF.T2 > ((int32_t) Config.InputsCfg.T2_Threshold * 10 - RD.Buttons.Int8_T2 * 10);
	in8_t2_bool = in8_t2_bool ^ Config.InputsCfg.T2_ThrMode;
	RD.Buttons.Int8_T2 = in8_t2_bool;
}

void RCC_user_init(void) {
	RCC->AHBENR |= RCC_AHBENR_CRCEN | RCC_AHBENR_DMA1EN;
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_TIM1EN;
	if (ARM_Core.Revision == 1) {
		//stm32
		RCC->CFGR |= RCC_CFGR_ADCPRE_0; //48 / 4 = 12mhz adc
		ADC_CLOCK = SystemCoreClock / 4;
	} else {
		//GD32 adc is much worse (about 8 times)
		RCC->CFGR |= RCC_CFGR_ADCPRE_0 | RCC_CFGR_ADCPRE_1; //48 / 8 = 6mhz adc
		ADC_CLOCK = SystemCoreClock / 8;
	}

	AFIO->MAPR = AFIO_MAPR_SWJ_CFG_1;
}

void GPIO_user_init(void) {
	AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_1 | AFIO_MAPR_TIM3_REMAP_PARTIALREMAP | AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1;

	GPIO_PIN_SETUP(P_IN1_Pin, P_IN1_GPIO_Port, GPIO_MODE_IN);
	GPIO_PIN_SETUP(P_IN2_Pin, P_IN2_GPIO_Port, GPIO_MODE_IN);
	GPIO_PIN_SETUP(P_IN3_Pin, P_IN3_GPIO_Port, GPIO_MODE_IN);
	GPIO_PIN_SETUP(P_IN4_Pin, P_IN4_GPIO_Port, GPIO_MODE_IN);
	GPIO_PIN_SETUP(P_IN5_Pin, P_IN5_GPIO_Port, GPIO_MODE_IN);
	GPIO_PIN_SETUP(P_IN6_Pin, P_IN6_GPIO_Port, GPIO_MODE_IN);

	GPIO_PIN_SETUP(USB_PULLUP_Pin, USB_PULLUP_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(P_USB_DP_Pin, P_USB_DP_GPIO_Port, GPIO_MODE_AN);

	GPIO_PIN_SETUP(ADC_12V_Pin, ADC_12V_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(ADC_T0_Pin, ADC_T0_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(ADC_T2_Pin, ADC_T2_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(ADC_T1_Pin, ADC_T1_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(ADC_BRAKE_Pin, ADC_BRAKE_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(ADC_THROTTLE_Pin, ADC_THROTTLE_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(ADC_AMP_Pin, ADC_AMP_GPIO_Port, GPIO_MODE_AN);

	if (Config.PWMouts.PWMIOmode == PWMIO_Mode_Open_drain) {
		GPIO_PIN_SETUP(P_OUT1_Pin, P_OUT1_GPIO_Port, GPIO_MODE_OUT10_ALT_OD);
		GPIO_PIN_SETUP(P_OUT2_Pin, P_OUT2_GPIO_Port, GPIO_MODE_OUT10_ALT_OD);
		GPIO_PIN_SETUP(P_OUT3_Pin, P_OUT3_GPIO_Port, GPIO_MODE_OUT10_ALT_OD);
		GPIO_PIN_SETUP(P_OUT4_USBDM_Pin, P_OUT4_USBDM_GPIO_Port, GPIO_MODE_OUT10_ALT_OD);
	} else if (Config.PWMouts.PWMIOmode == PWMIO_Mode_Push_Pull) {
		GPIO_PIN_SETUP(P_OUT1_Pin, P_OUT1_GPIO_Port, GPIO_MODE_OUT10_ALT_PP);
		GPIO_PIN_SETUP(P_OUT2_Pin, P_OUT2_GPIO_Port, GPIO_MODE_OUT10_ALT_PP);
		GPIO_PIN_SETUP(P_OUT3_Pin, P_OUT3_GPIO_Port, GPIO_MODE_OUT10_ALT_PP);
		GPIO_PIN_SETUP(P_OUT4_USBDM_Pin, P_OUT4_USBDM_GPIO_Port, GPIO_MODE_OUT10_ALT_PP);
	} else if (Config.PWMouts.PWMIOmode == PWMIO_Mode_USB) {
		GPIO_PIN_SETUP(P_OUT1_Pin, P_OUT1_GPIO_Port, GPIO_MODE_AN);
		GPIO_PIN_SETUP(P_OUT2_Pin, P_OUT2_GPIO_Port, GPIO_MODE_AN);
		GPIO_PIN_SETUP(P_OUT3_Pin, P_OUT3_GPIO_Port, GPIO_MODE_AN);
		GPIO_PIN_SETUP(P_OUT4_USBDM_Pin, P_OUT4_USBDM_GPIO_Port, GPIO_MODE_OUT50_ALT_PP);
		GPIO_PIN_SETUP(P_USB_DP_Pin, P_USB_DP_GPIO_Port, GPIO_MODE_OUT50_ALT_PP);

		GPIO_PIN_SETUP(USB_PULLUP_Pin, USB_PULLUP_GPIO_Port, GPIO_MODE_OUT10_PP);
		GPIO_SET_RESET(USB_PULLUP_Pin, USB_PULLUP_GPIO_Port, 1);
	} else {
		GPIO_PIN_SETUP(P_OUT1_Pin, P_OUT1_GPIO_Port, GPIO_MODE_AN);
		GPIO_PIN_SETUP(P_OUT2_Pin, P_OUT2_GPIO_Port, GPIO_MODE_AN);
		GPIO_PIN_SETUP(P_OUT3_Pin, P_OUT3_GPIO_Port, GPIO_MODE_AN);
		GPIO_PIN_SETUP(P_OUT4_USBDM_Pin, P_OUT4_USBDM_GPIO_Port, GPIO_MODE_AN);
	}

	GPIO_PIN_SETUP(PWM1_Pin, PWM1_GPIO_Port, GPIO_MODE_OUT10_ALT_PP); //TIM2
	GPIO_PIN_SETUP(PWM2_Pin, PWM2_GPIO_Port, GPIO_MODE_OUT10_ALT_PP);
	GPIO_PIN_SETUP(PWM3_Pin, PWM3_GPIO_Port, GPIO_MODE_OUT10_ALT_PP); //TIM3
	GPIO_PIN_SETUP(PWM4_Pin, PWM4_GPIO_Port, GPIO_MODE_OUT10_ALT_PP);
	GPIO_PIN_SETUP(PWM5_Pin, PWM5_GPIO_Port, GPIO_MODE_OUT10_ALT_PP); //TIM4
	GPIO_PIN_SETUP(PWM6_Pin, PWM6_GPIO_Port, GPIO_MODE_OUT10_ALT_PP);

	GPIO_PIN_SETUP(PWM_EN_Pin, PWM_EN_GPIO_Port, GPIO_MODE_OUT10_OD); //JTAG outputs enable
	//GPIO_SET_RESET(PWM_EN_Pin, PWM_EN_GPIO_Port);
	//GPIO_PIN_SETUP(PWM6_Pin, PWM6_GPIO_Port, GPIO_MODE_OUT10_PP);
	//GPIO_SET_RESET(PWM6_Pin, PWM6_GPIO_Port, 1);

	GPIO_PIN_SETUP(CAN_RX_Pin, CAN_RX_GPIO_Port, GPIO_MODE_IN);
	GPIO_PIN_SETUP(CAN_TX_Pin, CAN_TX_GPIO_Port, GPIO_MODE_OUT10_ALT_PP);
}

void GPIO_user_deinit(void) {
	//AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_1 | AFIO_MAPR_TIM3_REMAP_PARTIALREMAP | AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1;

	/*GPIO_PIN_SETUP(P_IN1_Pin, P_IN1_GPIO_Port, GPIO_MODE_IN);
	 GPIO_PIN_SETUP(P_IN2_Pin, P_IN2_GPIO_Port, GPIO_MODE_IN);
	 GPIO_PIN_SETUP(P_IN3_Pin, P_IN3_GPIO_Port, GPIO_MODE_IN);
	 GPIO_PIN_SETUP(P_IN4_Pin, P_IN4_GPIO_Port, GPIO_MODE_IN);
	 GPIO_PIN_SETUP(P_IN5_Pin, P_IN5_GPIO_Port, GPIO_MODE_IN);
	 GPIO_PIN_SETUP(P_IN6_Pin, P_IN6_GPIO_Port, GPIO_MODE_IN);

	 GPIO_PIN_SETUP(USB_PULLUP_Pin, USB_PULLUP_GPIO_Port, GPIO_MODE_AN);
	 GPIO_PIN_SETUP(P_USB_DP_Pin, P_USB_DP_GPIO_Port, GPIO_MODE_AN);*/

	GPIO_PIN_SETUP(P_OUT1_Pin, P_OUT1_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(P_OUT2_Pin, P_OUT2_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(P_OUT3_Pin, P_OUT3_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(P_OUT4_USBDM_Pin, P_OUT4_USBDM_GPIO_Port, GPIO_MODE_AN);

	GPIO_PIN_SETUP(PWM1_Pin, PWM1_GPIO_Port, GPIO_MODE_AN); //TIM2
	GPIO_PIN_SETUP(PWM2_Pin, PWM2_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(PWM3_Pin, PWM3_GPIO_Port, GPIO_MODE_AN); //TIM3
	GPIO_PIN_SETUP(PWM4_Pin, PWM4_GPIO_Port, GPIO_MODE_AN);
	GPIO_PIN_SETUP(PWM5_Pin, PWM5_GPIO_Port, GPIO_MODE_AN); //TIM4
	GPIO_PIN_SETUP(PWM6_Pin, PWM6_GPIO_Port, GPIO_MODE_AN);

	GPIO_PIN_SETUP(CAN_RX_Pin, CAN_RX_GPIO_Port, GPIO_MODE_IN);
	GPIO_PIN_SETUP(CAN_TX_Pin, CAN_TX_GPIO_Port, GPIO_MODE_OUT10_ALT_PP);
}

void EXTI_init(void) {
	//line 0 and 1
	EXTI->IMR |= 0x3;
	NVIC_SetPriority(EXTI0_IRQn, 2);	//CAN
	NVIC_SetPriority(EXTI1_IRQn, 3);	//MENU
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
}

void GoToBoot(uint8_t bootloader, int8_t node_id, int8_t server_id) {
	PWMDisableAll();
	GPIO_user_deinit();
	Storage_SaveData(Save_Lifetime);
	Storage_SaveData(Save_Data);

	BootCommand->GoToBoot = bootloader ? 0xDEADBEEF : 0;
	BootCommand->NodeID = node_id > 0 ? node_id : LC_Broadcast_Address;
	BootCommand->ServerID = server_id > 0 ? server_id : LC_Broadcast_Address;
	NVIC_SystemReset();
}

void SysTick_Handler() {
	static uint32_t delay = 0;
	SystemTick++;

	EXTI->SWIER = 1; //CAN

	delay++;
	if (delay >= 10) {
		EXTI->SWIER = 2; //MENU
		delay = 0;
	}

	ADC1->CR2 |= ADC_CR2_SWSTART;	// startup run
}

void EXTI0_IRQHandler(void) {
	EXTI->PR = 1;

	Network_Update(1);
}

void EXTI1_IRQHandler(void) {
	EXTI->PR = 2;

	updateButtons();

	LogicTick(10);

	if (RD.Menu.SWUpdate == 1) {
		RD.Menu.SWUpdate = 0;
		trace_printf("Reboot to boot\n");
		GoToBoot(1, LC_GetMyNodeName(0).NodeID, -1); //TODO add server id?
	}
	if (RD.Menu.Reboot == 1) {
		RD.Menu.Reboot = 0;
		trace_printf("Reboot\n");
		GoToBoot(0, -1, -1);
	}
	if (RD.Menu.Save) {
		Storage_SaveAll();
		RD.Menu.Save = 0;
		LC_EventSend(LevcanNodePtr, "Saved!", HWConfig.Name, LC_EB_Ok, LC_Broadcast_Address);
	}
	if (RD.Menu.LoadDefaults) {
		LoadDefaultParameters();
		RD.Menu.LoadDefaults = 0;
	}
	if (RD.Menu.WipeData) {
		Storage_Wipe();
		Storage_SaveData(Save_Lifetime);
		RD.Menu.WipeData = 0;
	}
	if (RD.Menu.ImportConf > 0 && RD.Menu.ImportConf < 10) {
		int num = RD.Menu.ImportConf;
		RD.Menu.ImportConf = 10;
		ImportConfig(num);
		RD.Menu.ImportConf = 0;
	}
	if (RD.Menu.ExportConf > 0 && RD.Menu.ImportConf < 10) {
		int num = RD.Menu.ExportConf;
		RD.Menu.ExportConf = 10;
		ExportConfig(num);
		RD.Menu.ExportConf = 0;
	}
}

void delay_ms(uint32_t delay) {
	uint32_t saved = SystemTick;
	while (SystemTick - saved < delay)
		;
}

void proceedSWU(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size) {
	(void) node;
	(void) size;
	switch (header.MsgID) {
	case LC_SYS_SWUpdate: {
		if (data && *((uint32_t*) data) == PROGRAM_KEY) {
			GoToBoot(1, LC_GetMyNodeName(0).NodeID, header.Source);
		}
	}
		break;
	}
}

#ifdef DEBUG
void assert_failed(char *file, uint32_t line) {
	trace_printf("Assert in: file %s on line %d\n", file, line);
}
#endif
