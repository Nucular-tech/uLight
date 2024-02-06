/*
 * directories.h
 *
 *  Created on: 12 февр. 2020 г.
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
const char conf_imports[] = LANG("?\n1\n2\n3\n4\n5\n6\n7\n8\n9\nLoading...", "?\n1\n2\n3\n4\n5\n6\n7\n8\n9\nЗагрузка...");
const char tsensors[] = "Off\nNTC10K3950\nNTC10K3380\nKTY84/130";
const char freqs[] = "100Hz\n500Hz\n1kHz\n5kHz\n10kHz\n24kHz(FAN)";
const char functions[] = LANG("OFF\nON\nTurnL\nTurnR\nBrk\nLBeam\nHBeam\nRev\nHorn\nTMot\nTCont\nT1\nT2\nS1\nS2\nS3\nS4", "Откл\nВкл\nПовЛ\nПовП\nТорм\nБлижС\nДалС\nЗ.ход\nГудок\nTМот\nTКонт\nT1\nT2\nS1\nS2\nS3\nS4");

// @formatter:off
LCPS_Entry_t PD_Root[]
= {
	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.Save , 	LANG("Save settings", "Сохранить настройки"),0 ),
	folder(LCP_AccessLvl_Any, 	Dir_Menu, 		0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_InputsConf, 0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_Func, 		0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_Outputs, 	0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_Inputs, 	0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_About, 		0, 0  ),
};

const LCPS_Entry_t PD_Menu[]
= {
	pstd(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.ImportConf, 			((LCP_Enum_t){0, 9}),	LANG("Import config.", "Импорт конфигурации"), conf_imports),
	pstd(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.ExportConf, 			((LCP_Enum_t){0, 9}),	LANG("Export config.", "Экспорт конфигурации"), conf_imports),
	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.Reboot , 				LANG("Reboot", "Перезагрузка"),0 ),
//	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.ResetWHusage , 			LANG("Reset Wh usage", "Сбросить расход Wh"),0 ),
//	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.ResetStats , 			LANG("Reset stats", "Сбросить статистику "),0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.LoadDefaults , 			LANG("Load defaults", "Сбросить настройки"),0 ),
//	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.WipeData , 				LANG("Erase data storage", "Стереть банки памяти"),0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,	LANG("# Export your settings #", "# Экспортируйте настройки #"),	0 ), //
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,	LANG("# before update! #", 		"# перед обновлением! #"),	0 ), //
	pbool(LCP_AccessLvl_Any, 	LCP_NLiveUpd,	RD.Menu.SWUpdate, 				LANG("Update firmware", "Обновить прошивку"),0 ),
};

LCPS_Entry_t PD_About[] = {
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		0,	0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("Firmware date", "Дата прошивки"), 0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("Firmware ver.", "Версия прошивки"), 0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("Loader date", "Дата загрузчика"), 0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("Loader version", "Версия загрузчика"), 0 ),
	//pstd(LCP_AccessLvl_Any, 	LCP_Normal | LCP_ReadOnly,		LifeData.PowerCycleCount,	zeroU32_t,	LANG("Power cycle", "Включений"), 0),
	//pstd(LCP_AccessLvl_Any, 	LCP_Normal | LCP_ReadOnly,		RD.Lifetime.Min,			zeroU32_t,	LANG("Power-on time", "Время работы"), LANG("%s min", "%d мин.")),
	//pstd(LCP_AccessLvl_Any, 	LCP_Normal | LCP_ReadOnly,		RD.Lifetime.Hours,			zeroU32_t,	"-", 	LANG("%s h", "%d ч.")),
	//pstd(LCP_AccessLvl_Any, 	LCP_Normal | LCP_ReadOnly,		RD.Lifetime.Days,			zeroU32_t,	"--", 	LANG("%s days", "%d дн.")),
};

const LCPS_Entry_t PD_Inputs[] = {
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.Amp,		((LCP_Decimal32_t){0,0,0,3}),	LANG("Current", "Ток"),			"%s A" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.V12,		((LCP_Decimal32_t){0,0,0,3}),	LANG("Voltage", "Напряжение"),	"%s V" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.VThrottle,	((LCP_Decimal32_t){0,0,0,3}),	LANG("Throttle", "Газ"),		"%s V" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.VBrake,		((LCP_Decimal32_t){0,0,0,3}),	LANG("Brake", "Тормоз"),		"%s V" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.Tint,		((LCP_Decimal32_t){0,0,0,1}),	LANG("T-sensor internal", "Термодатчик платы"),	"%s°C" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.T1,			((LCP_Decimal32_t){0,0,0,1}),	LANG("T-sensor T1", "Термодатчик T1"),	"%s°C" ),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		ADC_ValuesF.T2,			((LCP_Decimal32_t){0,0,0,1}),	LANG("T-sensor T2", "Термодатчик T2"),	"%s°C" ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int1, 		LANG("Input I1", "Вход I1"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int2, 		LANG("Input I2", "Вход I2"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int3, 		LANG("Input I3", "Вход I3"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int4, 		LANG("Input I4", "Вход I4"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int5, 		LANG("Input I5", "Вход I5"), 0 ),
	pbool(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,		RD.Buttons.Int6, 		LANG("Input I6", "Вход I6"), 0 ),
};

const LCPS_Entry_t PD_InputsConf[] = {
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T1, 					((LCP_Enum_t){0, Tsensor_MAX}),	LANG("T-sensor T1 type", "Тип датчика T1"), tsensors),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,	ADC_ValuesF.T1,							((LCP_Decimal32_t){0, 1, 1, 1}),LANG("# T-sensor T1", "# Термодатчик T1"),	"%s°C" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T1_Threshold,			((LCP_Uint32_t){0, 250, 1}),	LANG("T1 Threshold (port I7)", "T1 Порог (порт I7)"), "%d°C"),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T1_ThrMode, 			((LCP_Enum_t){0, 2}),			LANG("T1 Thr. mode", "T1 Режим порога"), LANG("Normal\nInverted","Норм.\nИнверт.")),

	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T2, 					((LCP_Enum_t){0,Tsensor_MAX}),	LANG("T-sensor T2 type", "Тип датчика T2"), tsensors),
	pstd(LCP_AccessLvl_Any, 	LCP_ROLiveUpd,	ADC_ValuesF.T2,							((LCP_Decimal32_t){0, 1, 1, 1}),LANG("# T-sensor T2", "# Термодатчик T2"),	"%s°C" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T2_Threshold,			((LCP_Uint32_t){0, 250, 1}),	LANG("T2 Threshold (port I8)", "T2 Порог (порт I8)"), "%d°C"),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.T2_ThrMode, 			((LCP_Enum_t){0, 2}),			LANG("T2 Thr. mode", "T2 Режим порога"), LANG("Normal\nInverted","Норм.\nИнверт.")),

	pbool(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.SendControl, 											LANG("CAN-Control", "CAN-Управление"), 0),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.InputsCfg.SendPorts, 			((LCP_Enum_t){0, 3}),			LANG("CAN-Ports", "CAN-Порты"), "OFF\nCAN 1-8\nCAN 9-16"  ),

	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,	LANG("# Logic buttons #", "# Логические кнопки #"),	0 ), //
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,	"# AND, NOT, OR, XOR #", 0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Logic.Input1a, 					((LCP_Enum_t){0, BtMax}),		LANG("Input A", "Вход А"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Logic.Input1b, 					((LCP_Enum_t){0, BtMax}),		LANG("Input B", "Вход Б"), buttons ),
};

#define dutymax   100
#define dutystep   5
const LCPS_Entry_t PD_Func[] = {
	pbool(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.AloneCANshutdown , 												LANG("Shutdown by CAN", "Отключение по CAN"),0),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Turn signal setup #", "# Настройка поворотников #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.LeftButton, 		((LCP_Enum_t){0, BtMax}),				LANG("Left turn switch", "Кнопка левого поворота"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.RightButton, 		((LCP_Enum_t){0, BtMax}),				LANG("Right turn switch", "Кнопка правого поворота"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.WarningButton, 	((LCP_Enum_t){0, BtMax}),				LANG("Warning switch", "Аварийная кнопка"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.HighDuty,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn brightness", "Яркость поворотников"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.LowDuty ,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn off-brightness", "Яркость выкл. поворот."), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.OnTime,			((LCP_Decimal32_t){1, 120, 1, 1}),		LANG("Turn on-time", "Время вкл. поворотников"), "%ssec" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Turns.OffTime,			((LCP_Decimal32_t){0, 120, 1, 1}),		LANG("Turn off-time", "Время выкл. поворотников"), "%ssec" ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Brake signal setup #", "# Настройка тормоза #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Brake.Button, 			((LCP_Enum_t){0, BtMax}),				LANG("Brake button", "Кнопка тормоза"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Brake.LowBrakeDuty,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Off-brake brightness", "Яркость без торможения"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Brake.HighBrakeDuty,	((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("On-brake brightness", "Яркость торможения"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Brake.StrobePeriod,		((LCP_Decimal32_t){2, 50, 1, 2}),		LANG("Brake strobe period", "Период мигания тормоза"), "%ssec" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Brake.StrobeCount,		((LCP_Uint32_t){0, 10, 1}),				LANG("Brake strobe count", "Кол-во миганий тормоза"), 0 ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Reverse signal setup #", "# Настройка заднего хода #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Reverse.LowDuty,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Off-reverse bright.", "Яркость без з.хода"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Reverse.HighDuty,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("On-reverse bright.", "Яркость заднего хода"), "%d%%" ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Beam setup #", "# Настройка фар #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.LowBeamButton, 	((LCP_Enum_t){0, BtMax}),				LANG("Low beam switch", "Кнопка ближнего света"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.HighBeamButton, 	((LCP_Enum_t){0, BtMax}),				LANG("High beam switch", "Кнопка дальнего света"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.BeamsMode, 		((LCP_Enum_t){0, 2}),					LANG("Beam mode", "Режим фары"), LANG("Separate\nSingle", "Раздельный\nОбщий") ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.MinDuty,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Beam min brightness", "Мин. яркость фары"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.LowDuty,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Low beam brightness", "Яркость ближнего света"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Beam.HighDuty,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("High beam brightness", "Яркость дальнего света"), "%d%%" ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Horn setup #", "# Настройка гудка #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Horn.HornButton, 		((LCP_Enum_t){0, BtMax}),				LANG("Horn switch", "Кнопка гудка"), buttons ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Signal S1-S4 #", "# Сигнал S1-S4 #"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.B1, 				((LCP_Enum_t){0, BtMax}),				LANG("Button S1", "Кнопка S1"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOn1,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn on-brightness S1", "Яркость вкл. S1"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOff1,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn off-brightness S1", "Яркость выкл. S1"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.B2, 				((LCP_Enum_t){0, BtMax}),				LANG("Button S2", "Кнопка S2"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOn2,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn on-brightness S2", "Яркость вкл. S2"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOff2,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn off-brightness S2", "Яркость выкл. S2"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.B3, 				((LCP_Enum_t){0, BtMax}),				LANG("Button S3", "Кнопка S3"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOn3,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn on-brightness S3", "Яркость вкл. S3"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOff3,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn off-brightness S3", "Яркость выкл. S3"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.B4, 				((LCP_Enum_t){0, BtMax}),				LANG("Button S4", "Кнопка S4"), buttons ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOn4,			((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn on-brightness S4", "Яркость вкл. S4"), "%d%%" ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.Signal.DutyOff4,		((LCP_Uint32_t){0,dutymax,dutystep}),	LANG("Turn off-brightness S4", "Яркость выкл. S4"), "%d%%" ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,		LANG("# Fan control #", "# Настройка вентиляторов #"),	0 ), //
	folder(LCP_AccessLvl_Any, 	Dir_TsFunctions1, 0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_TsFunctions2, 0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_TsFunctions3, 0, 0  ),
	folder(LCP_AccessLvl_Any, 	Dir_TsFunctions4, 0, 0  ),
};

const LCPS_Entry_t PD_TsFunctions[] = {
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.FanConrol.Tmin[0],			((LCP_Int32_t){ -30, 125, 5}),		LANG("Temperature min", "Мин. температура"),"%d°C"),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.FanConrol.Tmax[0],			((LCP_Int32_t){ -30, 125, 5}),		LANG("Temperature max", "Макс. температура"),"%d°C"),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.FanConrol.OutMin[0],		((LCP_Uint32_t){ 0, 100, 5}),		LANG("PWM output min", "ШИМ выход мин."),"%d%%"),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.Func.FanConrol.OutMax[0],		((LCP_Uint32_t){ 0, 100, 5}),		LANG("PWM output max", "ШИМ выход макс."),"%d%%"),
};


const LCPS_Entry_t PD_Outputs[] = {
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,										LANG("# Reboot to apply", "#Перезагрузите чтобы применить"),	0 ), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM1_2freq, 			((LCP_Enum_t){0, 6}),				LANG("PWM 1-2 freq ", "Частота ШИМ 1-2"), freqs ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM1out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 1", "Выход 1"), functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM2out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 2", "Выход 2"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM3_4freq, 			((LCP_Enum_t){0, 6}),				LANG("PWM 3-4 freq ", "Частота ШИМ 3-4"), freqs ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM3out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 3", "Выход 3"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM4out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 4", "Выход 4"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM5_6freq, 			((LCP_Enum_t){0, 6}),				LANG("PWM 5-6 freq ", "Частота ШИМ 5-6"), freqs ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM5out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 5", "Выход 5"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWM6out, 			((LCP_Enum_t){0, Func_MAX}),		LANG("Output 6", "Выход 6"), 	functions ),
	label(LCP_AccessLvl_Any, 	LCP_ReadOnly,										LANG("# Reboot to apply", "#Перезагрузите чтобы применить"),	0), //
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWMIOmode, 			((LCP_Enum_t){0, 3}),				LANG("PWM IO mode", "Режим ШИМ IO"),"OFF\nOpen-drain\nPush-Pull\nUSB"), //USB is not supported actually
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.PWMIOfreq, 			((LCP_Enum_t){0, 6}),				LANG("PWM IO freq", "Частота ШИМ IO"), freqs ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.IO1out, 				((LCP_Enum_t){0, Func_MAX}),		LANG("Output P1", "Выход P1"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.IO2out, 				((LCP_Enum_t){0, Func_MAX}),		LANG("Output P2", "Выход P2"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.IO3out, 				((LCP_Enum_t){0, Func_MAX}),		LANG("Output P3", "Выход P3"), 	functions ),
	pstd(LCP_AccessLvl_Any, 	LCP_Normal,		Config.PWMouts.IO4out, 				((LCP_Enum_t){0, Func_MAX}),		LANG("Output P4", "Выход P4"), 	functions ),
};

//All directories and it's names
const LCPS_Directory_t PD_Directories[] = {
	directory(PD_Root, 0, LCP_AccessLvl_Any, 			LANG("uLight", "микроЛайт")),
	directory(PD_Menu, 0, LCP_AccessLvl_Any, 			LANG("Updates and settings", "Настройки и обновление")),
	directory(PD_About, 0, LCP_AccessLvl_Any, 			LANG("Device information", "Информация об устройстве")),
	directory(PD_Inputs, 0, LCP_AccessLvl_Any, 			LANG("Input values", "Значения входов")),
	directory(PD_Outputs, 0, LCP_AccessLvl_Any,			LANG("Output configuration", "Настройка выходов")),
	directory(PD_Func, 0, LCP_AccessLvl_Any, 			LANG("Functions setup", "Настройка функций")),
	directory(PD_InputsConf, 0, LCP_AccessLvl_Any, 		LANG("Inputs configuration", "Настройка входов") ),
	directory(PD_TsFunctions, TsensFunc_Motor, LCP_AccessLvl_Any,	LANG("Motor T-sensor", "Т-датчик мотора")),
	directory(PD_TsFunctions, TsensFunc_Contr, LCP_AccessLvl_Any,	LANG("Controller T-sensor", "Т-датчик контроллера")),
	directory(PD_TsFunctions, TsensFunc_T1, LCP_AccessLvl_Any,		LANG("T1 T-sensor", "Т-датчик T1")),
	directory(PD_TsFunctions, TsensFunc_T2, LCP_AccessLvl_Any,		LANG("T2 T-sensor", "Т-датчик T2")),
};
// @formatter:on
const uint32_t PD_Directories_size = sizeof(PD_Directories) / sizeof(PD_Directories[0]);
