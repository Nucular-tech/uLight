/*
 * directories.h
 *
 *  Created on: 12 ����. 2020 �.
 *      Author: VasiliSk
 */

#include "levcan.h"
#include "levcan_paramserver.h"
#include "parameters.h"
#include "adc.h"

//Pointers to reference
extern LCPS_Entry_t PD_Root[], PD_About[];
extern const LCPS_Entry_t PD_Inputs[], PD_Outputs[], PD_Menu[], PD_Func[], PD_InputsConf[], PD_TsFunctions[];

uint32_t dummy;

#ifdef ENGLISH
#define LANG(en, ru) en
#endif
#ifdef RUSSIAN
#define LANG(en, ru) ru
#endif

#ifdef __CDT_PARSER__
#define _Generic(...) 0
#endif
#define LCP_ROLiveUpd (LCP_ReadOnly | LCP_LiveUpdate)
#define LCP_NLiveUpd (LCP_Normal | LCP_LiveUpdate)

const LCP_Uint32_t zeroU32_t = { 0, 0, 0 };
const char buttons[] = "Off\nON\nI1\nI2\nI3\nI4\nI5\nI6\nI7_T1\nI8_T2\nC1\nC2\nC3\nC4\nC5\nC6\nC7\nC8\nC9\nC10\nC11\nC12\nC13\nC14\nC15\nC16\nBrk\nAND\nNOT\nOR\nXOR";
const char conf_imports[] = LANG("?\n1\n2\n3\n4\n5\n6\n7\n8\n9\nLoading...", "?\n1\n2\n3\n4\n5\n6\n7\n8\n9\n��������...");
const char tsensors[] = "Off\nNTC10K3950\nNTC10K3380\nKTY84/130";
const char freqs[] = "100Hz\n500Hz\n1kHz\n5kHz\n10kHz\n24kHz(FAN)";
const char functions[] = LANG("OFF\nON\nTurnL\nTurnR\nBrk\nLBeam\nHBeam\nRev\nHorn\nTMot\nTCont\nT1\nT2\nS1\nS2\nS3\nS4",
		"����\n���\n����\n����\n����\n�����\n����\n�.���\n�����\nT���\nT����\nT1\nT2\nS1\nS2\nS3\nS4");

// @formatter:off
LCPS_Entry_t PD_Root[]
= {
	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.Save , 	LANG("Save settings", "��������� ���������"),0 ),
	folder(LCP_AccessLvl_Any, 	Dir_Menu, 		0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_InputsConf, 0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_Func, 		0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_Outputs, 	0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_Inputs, 	0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_About, 		0, 0  ),
};

const LCPS_Entry_t PD_Menu[]
= {
	pstd(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.ImportConf, 			((LCP_Enum_t){0, 9}),	LANG("Import config.", "������ ������������"), conf_imports),
	pstd(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.ExportConf, 			((LCP_Enum_t){0, 9}),	LANG("Export config.", "������� ������������"), conf_imports),
	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.Reboot , 				LANG("Reboot", "������������"),0 ),
//	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.ResetWHusage , 			LANG("Reset Wh usage", "�������� ������ Wh"),0 ),
//	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.ResetStats , 			LANG("Reset stats", "�������� ���������� "),0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.LoadDefaults , 			LANG("Load defaults", "�������� ���������"),0 ),
//	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.WipeData , 				LANG("Erase data storage", "������� ����� ������"),0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,	LANG("# Export your settings #", "# ������������� ��������� #"),	0 ), //
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,	LANG("# before update! #", 		"# ����� �����������! #"),	0 ), //
	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.SWUpdate, 				LANG("Update firmware", "�������� ��������"),0 ),
};

LCPS_Entry_t PD_About[] = {
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		0,	0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("Firmware date", "���� ��������"), 0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("Firmware ver.", "������ ��������"), 0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("Loader date", "���� ����������"), 0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("Loader version", "������ ����������"), 0 ),
	//pstd(LCP_AccessLvl_Any, 	LCP_Normal | LCP_ReadOnly,		LifeData.PowerCycleCount,	zeroU32_t,	LANG("Power cycle", "���������"), 0),
	//pstd(LCP_AccessLvl_Any, 	LCP_Normal | LCP_ReadOnly,		RD.Lifetime.Min,			zeroU32_t,	LANG("Power-on time", "����� ������"), LANG("%s min", "%d ���.")),
	//pstd(LCP_AccessLvl_Any, 	LCP_Normal | LCP_ReadOnly,		RD.Lifetime.Hours,			zeroU32_t,	"-", 	LANG("%s h", "%d �.")),
	//pstd(LCP_AccessLvl_Any, 	LCP_Normal | LCP_ReadOnly,		RD.Lifetime.Days,			zeroU32_t,	"--", 	LANG("%s days", "%d ��.")),
};
extern uint8_t can_exbuttons[];
const LCPS_Entry_t PD_Inputs[] = {
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.Amp,		((LCP_Decimal32_t){0,0,0,3}),	LANG("Current", "���"),			"%s A" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.V12,		((LCP_Decimal32_t){0,0,0,3}),	LANG("Voltage", "����������"),	"%s V" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.VThrottle,	((LCP_Decimal32_t){0,0,0,3}),	LANG("Throttle", "���"),		"%s V" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.VBrake,		((LCP_Decimal32_t){0,0,0,3}),	LANG("Brake", "������"),		"%s V" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.Tint,		((LCP_Decimal32_t){0,0,0,1}),	LANG("T-sensor internal", "����������� �����"),	"%s�C" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.T1,			((LCP_Decimal32_t){0,0,0,1}),	LANG("T-sensor T1", "����������� T1"),	"%s�C" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.T2,			((LCP_Decimal32_t){0,0,0,1}),	LANG("T-sensor T2", "����������� T2"),	"%s�C" ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int1, 		LANG("Input I1", "���� I1"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int2, 		LANG("Input I2", "���� I2"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int3, 		LANG("Input I3", "���� I3"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int4, 		LANG("Input I4", "���� I4"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int5, 		LANG("Input I5", "���� I5"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int6, 		LANG("Input I6", "���� I6"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[0], 	LANG("CAN I1", "CAN I1"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[1], 	LANG("CAN I2", "CAN I2"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[2], 	LANG("CAN I3", "CAN I3"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[3], 	LANG("CAN I4", "CAN I4"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[4], 	LANG("CAN I5", "CAN I5"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[5], 	LANG("CAN I6", "CAN I6"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[6], 	LANG("CAN I7", "CAN I7"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[7], 	LANG("CAN I8", "CAN I8"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[8], 	LANG("CAN I9", "CAN I9"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[9], 	LANG("CAN I10", "CAN I10"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[10], 	LANG("CAN I11", "CAN I11"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[11], 	LANG("CAN I12", "CAN I12"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[12], 	LANG("CAN I13", "CAN I13"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[13], 	LANG("CAN I14", "CAN I14"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[14], 	LANG("CAN I15", "CAN I15"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd, 		can_exbuttons[15], 	LANG("CAN I16", "CAN I16"), 0 ),


};

const LCPS_Entry_t PD_InputsConf[] = {
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.InputFilter,			((LCP_Decimal32_t){0, 20, 1, 2}),LANG("Inputs filter", "������ ������"),	"%ss" ),

	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T1, 					((LCP_Enum_t){0, Tsensor_MAX}),	LANG("T-sensor T1 type", "��� ������� T1"), tsensors),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,	ADC_ValuesF.T1,							((LCP_Decimal32_t){0, 1, 1, 1}),LANG("# T-sensor T1", "# ����������� T1"),	"%s�C" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T1_Threshold,			((LCP_Uint32_t){0, 250, 1}),	LANG("T1 Threshold (port I7)", "T1 ����� (���� I7)"), "%d�C"),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T1_ThrMode, 			((LCP_Enum_t){0, 2}),			LANG("T1 Thr. mode", "T1 ����� ������"), LANG("Normal\nInverted","����.\n������.")),

	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T2, 					((LCP_Enum_t){0,Tsensor_MAX}),	LANG("T-sensor T2 type", "��� ������� T2"), tsensors),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,	ADC_ValuesF.T2,							((LCP_Decimal32_t){0, 1, 1, 1}),LANG("# T-sensor T2", "# ����������� T2"),	"%s�C" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T2_Threshold,			((LCP_Uint32_t){0, 250, 1}),	LANG("T2 Threshold (port I8)", "T2 ����� (���� I8)"), "%d�C"),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T2_ThrMode, 			((LCP_Enum_t){0, 2}),			LANG("T2 Thr. mode", "T2 ����� ������"), LANG("Normal\nInverted","����.\n������.")),

	pbool(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.SendControl, 											LANG("CAN-Control", "CAN-����������"), 0),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.SendPorts, 			((LCP_Enum_t){0, 3}),			LANG("CAN-Ports", "CAN-�����"), "OFF\nCAN 1-8\nCAN 9-16"  ),

	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,	LANG("# Logic buttons #", "# ���������� ������ #"),	0 ), //
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,	"# AND, NOT, OR, XOR #", 0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Logic.Input1a, 					((LCP_Enum_t){0, BtMax-4}),		LANG("Input A", "���� �"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Logic.Input1b, 					((LCP_Enum_t){0, BtMax-4}),		LANG("Input B", "���� �"), buttons ),
};

#define dutymax   100
#define dutystep   5
const LCPS_Entry_t PD_Func[] = {
	pbool(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.ShutdownByCAN , 												LANG("Shutdown by CAN", "���������� �� CAN"),0),
	pbool(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.ShutdownOnCharge , 												LANG("Shutdown on charge", "���������� ��� �������"),0),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.ShutdownByButton, 		((LCP_Enum_t){0, BtMax}),				LANG("Shutdown button", "������ ����������"), buttons ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Turn signal setup #", "# ��������� ������������ #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.LeftButton, 		((LCP_Enum_t){0, BtMax}),				LANG("Left turn switch", "������ ������ ��������"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.RightButton, 		((LCP_Enum_t){0, BtMax}),				LANG("Right turn switch", "������ ������� ��������"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.WarningButton, 	((LCP_Enum_t){0, BtMax}),				LANG("Warning switch", "��������� ������"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.HighDuty,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn brightness", "������� ������������"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.LowDuty ,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn off-brightness", "������� ����. �������."), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.OnTime,			((LCP_Decimal32_t){1, 120, 1, 1}),		LANG("Turn on-time", "����� ���. ������������"), "%ssec" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.OffTime,			((LCP_Decimal32_t){0, 120, 1, 1}),		LANG("Turn off-time", "����� ����. ������������"), "%ssec" ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Brake signal setup #", "# ��������� ������� #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Brake.Button, 			((LCP_Enum_t){0, BtMax}),				LANG("Brake button", "������ �������"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Brake.LowBrakeDuty,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Off-brake brightness", "������� ��� ����������"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Brake.HighBrakeDuty,	((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("On-brake brightness", "������� ����������"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Brake.StrobePeriod,		((LCP_Decimal32_t){2, 50, 1, 2}),		LANG("Brake strobe period", "������ ������� �������"), "%ssec" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Brake.StrobeCount,		((LCP_Uint32_t){0, 10, 1}),				LANG("Brake strobe count", "���-�� ������� �������"), 0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Reverse signal setup #", "# ��������� ������� ���� #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Reverse.LowDuty,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Off-reverse bright.", "������� ��� �.����"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Reverse.HighDuty,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("On-reverse bright.", "������� ������� ����"), "%d%%" ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Beam setup #", "# ��������� ��� #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.LowBeamButton, 	((LCP_Enum_t){0, BtMax}),				LANG("Low beam switch", "������ �������� �����"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.HighBeamButton, 	((LCP_Enum_t){0, BtMax}),				LANG("High beam switch", "������ �������� �����"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.BeamsMode, 		((LCP_Enum_t){0, 2}),					LANG("Beam mode", "����� ����"), LANG("Separate\nSingle", "����������\n�����") ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.MinDuty,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Beam min brightness", "���. ������� ����"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.LowDuty,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Low beam brightness", "������� �������� �����"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.HighDuty,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("High beam brightness", "������� �������� �����"), "%d%%" ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Horn setup #", "# ��������� ����� #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Horn.HornButton, 		((LCP_Enum_t){0, BtMax}),				LANG("Horn switch", "������ �����"), buttons ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Signal S1-S4 #", "# ������ S1-S4 #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.B1, 				((LCP_Enum_t){0, BtMax}),				LANG("Button S1", "������ S1"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOn1,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn on-brightness S1", "������� ���. S1"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOff1,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn off-brightness S1", "������� ����. S1"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.B2, 				((LCP_Enum_t){0, BtMax}),				LANG("Button S2", "������ S2"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOn2,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn on-brightness S2", "������� ���. S2"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOff2,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn off-brightness S2", "������� ����. S2"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.B3, 				((LCP_Enum_t){0, BtMax}),				LANG("Button S3", "������ S3"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOn3,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn on-brightness S3", "������� ���. S3"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOff3,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn off-brightness S3", "������� ����. S3"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.B4, 				((LCP_Enum_t){0, BtMax}),				LANG("Button S4", "������ S4"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOn4,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn on-brightness S4", "������� ���. S4"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOff4,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn off-brightness S4", "������� ����. S4"), "%d%%" ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Fan control #", "# ��������� ������������ #"),	0 ), //
	folder(LCP_AccessLvl_Any, 	Dir_TsFunctions1, 0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_TsFunctions2, 0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_TsFunctions3, 0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_TsFunctions4, 0, 0  ),
};

const LCPS_Entry_t PD_TsFunctions[] = {
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.FanConrol.Tmin[0],			((LCP_Int32_t){ -30, 125, 5}),		LANG("Temperature min", "���. �����������"),"%d�C"),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.FanConrol.Tmax[0],			((LCP_Int32_t){ -30, 125, 5}),		LANG("Temperature max", "����. �����������"),"%d�C"),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.FanConrol.OutMin[0],		((LCP_Uint32_t){ 0, 100, 5}),		LANG("PWM output min", "��� ����� ���."),"%d%%"),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.FanConrol.OutMax[0],		((LCP_Uint32_t){ 0, 100, 5}),		LANG("PWM output max", "��� ����� ����."),"%d%%"),
};


const LCPS_Entry_t PD_Outputs[] = {
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,										LANG("# Reboot to apply", "#������������� ����� ���������"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM1_2freq, 			((LCP_Enum_t){0, 6}),				LANG("PWM 1-2 freq ", "������� ��� 1-2"), freqs ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM1out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 1", "����� 1"), functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM2out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 2", "����� 2"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM3_4freq, 			((LCP_Enum_t){0, 6}),				LANG("PWM 3-4 freq ", "������� ��� 3-4"), freqs ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM3out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 3", "����� 3"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM4out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 4", "����� 4"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM5_6freq, 			((LCP_Enum_t){0, 6}),				LANG("PWM 5-6 freq ", "������� ��� 5-6"), freqs ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM5out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 5", "����� 5"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM6out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 6", "����� 6"), 	functions ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,										LANG("# Reboot to apply", "#������������� ����� ���������"),	0), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWMIOmode, 			((LCP_Enum_t){0, 3}),				LANG("PWM IO mode", "����� ��� IO"),"OFF\nOpen-drain\nPush-Pull\nUSB"), //USB is not supported actually
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWMIOfreq, 			((LCP_Enum_t){0, 6}),				LANG("PWM IO freq", "������� ��� IO"), freqs ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.IO1out, 				((LCP_Enum_t){0, Func_MAX}),		LANG("Output P1", "����� P1"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.IO2out, 				((LCP_Enum_t){0, Func_MAX}),		LANG("Output P2", "����� P2"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.IO3out, 				((LCP_Enum_t){0, Func_MAX}),		LANG("Output P3", "����� P3"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.IO4out, 				((LCP_Enum_t){0, Func_MAX}),		LANG("Output P4", "����� P4"), 	functions ),
};

//All directories and it's names
const LCPS_Directory_t PD_Directories[] = {
	directory(PD_Root, 0, LCP_AccessLvl_Any, 			LANG("uLight", "���������")),
	directory(PD_Menu, 0, LCP_AccessLvl_Any, 			LANG("Updates and settings", "��������� � ����������")),
	directory(PD_About, 0, LCP_AccessLvl_Any, 			LANG("Device information", "���������� �� ����������")),
	directory(PD_Inputs, 0, LCP_AccessLvl_Any, 			LANG("Input values", "�������� ������")),
	directory(PD_Outputs, 0, LCP_AccessLvl_Any,			LANG("Output configuration", "��������� �������")),
	directory(PD_Func, 0, LCP_AccessLvl_Any, 			LANG("Functions setup", "��������� �������")),
	directory(PD_InputsConf, 0, LCP_AccessLvl_Any, 		LANG("Inputs configuration", "��������� ������") ),
	directory(PD_TsFunctions, TsensFunc_Motor, LCP_AccessLvl_Any,	LANG("Motor T-sensor", "�-������ ������")),
	directory(PD_TsFunctions, TsensFunc_Contr, LCP_AccessLvl_Any,	LANG("Controller T-sensor", "�-������ �����������")),
	directory(PD_TsFunctions, TsensFunc_T1, LCP_AccessLvl_Any,		LANG("T1 T-sensor", "�-������ T1")),
	directory(PD_TsFunctions, TsensFunc_T2, LCP_AccessLvl_Any,		LANG("T2 T-sensor", "�-������ T2")),
};
// @formatter:on
const uint32_t PD_Directories_size = sizeof(PD_Directories) / sizeof(PD_Directories[0]);
