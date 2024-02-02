/*
 * parameters.h
 *
 *  Created on: 12 feb 2020 ã.
 *      Author: VasiliSk
 */

#include "levcan_objects.h"

#pragma once

typedef enum {
	Dir_ROOT,
	Dir_Menu,
	Dir_About,
	Dir_Inputs,
	Dir_Outputs,
	Dir_Func,
	Dir_InputsConf,
	Dir_TsFunctions1,
	Dir_TsFunctions2,
	Dir_TsFunctions3,
	Dir_TsFunctions4,
	Dir_END = 127
} Directories_t;

enum {
	Save_Data, Save_Param, Save_Lifetime, Save_number
};

typedef enum {
	BtDisabled,
	BtEnabled,
	BtInt_1,
	BtInt_2,
	BtInt_3,
	BtInt_4,
	BtInt_5,
	BtInt_6,
	BtInt_7_T1,
	BtInt_8_T2,
	BtExt_1,
	BtExt_2,
	BtExt_3,
	BtExt_4,
	BtExt_5,
	BtExt_6,
	BtExt_7,
	BtExt_8,
	BtExt_9,
	BtExt_10,
	BtExt_11,
	BtExt_12,
	BtExt_13,
	BtExt_14,
	BtExt_15,
	BtExt_16,
	BtAnd_1,
	BtNot_1,
	BtOr_1,
	BtXor_1,
	BtMax,
} Buttons_t;

typedef enum {
	Func_Off,
	Func_On,
	Func_TurnLeft,
	Func_TurnRight,
	Func_Brake,
	Func_LowBeam,
	Func_HighBeam,
	Func_Reverse,
	Func_Horn,
	Func_MotorT,
	Func_ContrT,
	Func_T1,
	Func_T2,
	Func_Signal1,
	Func_Signal2,
	Func_Signal3,
	Func_Signal4,
	Func_MAX,
} Function_t;

typedef enum {
	Tsensor_OFF, Tsensor_NTC10K3950, Tsensor_NTC10K3380, Tsensor_KTY84_130, Tsensor_MAX
} Tsensor_t;

typedef enum {
	TthrMode_Normal, TthrMode_Inverted
} TthrMode_t;

typedef enum {
	TsensFunc_Motor, TsensFunc_Contr, TsensFunc_T1, TsensFunc_T2, TsensFunc_MAX
} TsensFunc_t;

typedef enum {
	PWMIO_Mode_OFF, PWMIO_Mode_Open_drain, PWMIO_Mode_Push_Pull, PWMIO_Mode_USB
} PWMIO_Mode_t;

typedef struct {
	struct {

		struct {
			Buttons_t LeftButton;
			Buttons_t RightButton;
			Buttons_t WarningButton;
			uint8_t HighDuty;
			uint8_t LowDuty;
			uint8_t OffTime;
			uint8_t OnTime;
		} Turns;

		struct {
			Buttons_t HornButton;
			uint8_t Strobe;
		} Horn;

		struct {
			uint8_t LowBrakeDuty;
			uint8_t HighBrakeDuty;
			uint8_t StrobePeriod;
			uint8_t StrobeCount;
			uint8_t Button;
		} Brake;

		struct {
			uint8_t LowDuty;
			uint8_t HighDuty;
		} Reverse;

		struct {
			Buttons_t LowBeamButton;
			Buttons_t HighBeamButton;
			enum {
				Beams_Separate, Beams_Single
			} BeamsMode;
			uint8_t MinDuty;
			uint8_t LowDuty;
			uint8_t HighDuty;
		} Beam;

		struct {
			int8_t Tmin[TsensFunc_MAX];
			int8_t Tmax[TsensFunc_MAX];
			uint8_t OutMin[TsensFunc_MAX];
			uint8_t OutMax[TsensFunc_MAX];
		} FanConrol;

		struct {
			Buttons_t B1;
			Buttons_t B2;
			Buttons_t B3;
			Buttons_t B4;
			uint8_t DutyOn1;
			uint8_t DutyOff1;
			uint8_t DutyOn2;
			uint8_t DutyOff2;
			uint8_t DutyOn3;
			uint8_t DutyOff3;
			uint8_t DutyOn4;
			uint8_t DutyOff4;
		} Signal;
		uint8_t AloneCANshutdown;
	} Func;

	union {
		uint8_t PWMoutArray[15];
		struct { //Function_t
			uint8_t PWM1out;
			uint8_t PWM2out;
			uint8_t PWM3out;
			uint8_t PWM4out;
			uint8_t PWM5out;
			uint8_t PWM6out;
			uint8_t IO1out;
			uint8_t IO2out;
			uint8_t IO3out;
			uint8_t IO4out;
			uint8_t PWM1_2freq;
			uint8_t PWM3_4freq;
			uint8_t PWM5_6freq;
			uint8_t PWMIOfreq;
			uint8_t PWMIOmode;
		};
	} PWMouts;

	struct {
		Tsensor_t T1;
		Tsensor_t T2;
		uint8_t T1_Threshold;
		uint8_t T2_Threshold;
		TthrMode_t T1_ThrMode;
		TthrMode_t T2_ThrMode;

		uint8_t SendControl;
		uint8_t SendPorts;
		uint16_t ThrottleMin;
		uint16_t ThrottleMax;
		uint16_t BrakeMin;
		uint16_t BrakeMax;
	} InputsCfg;

	struct{
		Buttons_t Input1a;
		Buttons_t Input1b;
	} Logic;

} ConfigStruct_t;

typedef struct {

} DataStruct_t;

typedef struct {
	uint32_t TotalOnTime; //sec
	uint32_t PowerCycleCount; //enable times
} LifeDataStruct_t;

typedef struct {
	struct {
		uint8_t Reboot;
		uint8_t LoadDefaults;
		uint8_t WipeData;
		uint8_t Save;
		uint8_t SWUpdate;
		uint8_t ImportConf;
		uint8_t ExportConf;
	} Menu;

	struct {
		uint16_t Min;
		uint16_t Hours;
		uint16_t Days;
	} Lifetime;

	union {
		uint8_t ButtonArray[BtMax];
		struct {
			uint8_t OffBt;
			uint8_t OnBt;
			uint8_t Int1;
			uint8_t Int2;
			uint8_t Int3;
			uint8_t Int4;
			uint8_t Int5;
			uint8_t Int6;
			uint8_t Int7_T1;
			uint8_t Int8_T2;
			uint8_t Ext1;
			uint8_t Ext2;
			uint8_t Ext3;
			uint8_t Ext4;
			uint8_t Ext5;
			uint8_t Ext6;
			uint8_t Ext7;
			uint8_t Ext8;
			uint8_t Ext9;
			uint8_t Ext10;
			uint8_t Ext11;
			uint8_t Ext12;
			uint8_t Ext13;
			uint8_t Ext14;
			uint8_t Ext15;
			uint8_t Ext16;
			uint8_t AND1;
			uint8_t NOT1;
			uint8_t OR1;
			uint8_t XOR1;
		};
	} Buttons;

} RuntimeStruct_t;

extern volatile ConfigStruct_t Config;
extern volatile DataStruct_t Data;
extern volatile LifeDataStruct_t LifeData;
extern volatile RuntimeStruct_t RD;
extern const uint16_t PWMIO_Freq[];

void LoadDefaultParameters(void);
void LoadStorage(void);
void ExportConfig(int index);
void ImportConfig(int index);
