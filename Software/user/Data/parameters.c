#include <ctype.h>
#include <Data/directories.h>
#include <Data/parameters.h>
#include <levcan.h>
#include <levcan_events.h>
#include <levcan_fileclient.h>
#include <levcan_filedef.h>
#include <levcan_paramcommon.h>
#include <levcan_paramserver.h>
#include <network.h>
#include <Resconfig.h>
#include <stdio.h>
#include <storage.h>
#include <string.h>
#include <sys/_stdint.h>
#include <sys/types.h>

//number of pages in bank
#define FLASH_ADDRES_BANK0  (intptr_t*)(&__storage_start__)
#define FLASH_ADDRES_BANK1 (intptr_t*)((uint32_t)(&__storage_start__) + FLASH_PAGE_SIZE_DATA * FLASH_PAGES_IN_BANK)
//ld scripts global varibles
extern unsigned int __storage_start__;

volatile ConfigStruct_t Config;
volatile DataStruct_t Data;
volatile LifeDataStruct_t LifeData;
volatile RuntimeStruct_t RD = { 0 };

char rdata[512];
// @formatter:off
const StorageData_t Storagedata[Struct_number] = { //
		{ sizeof(Data),		 	1, (intptr_t*) &Data }, //
		{ sizeof(Config), 		3, (intptr_t*) &Config }, //
		{ sizeof(LifeData), 	0, (intptr_t*) &LifeData }, //
};
// @formatter:on

const uint16_t PWMIO_Freq[] = { 100, 500, 1000, 5000, 10000, 24000 };

void LoadDefaultParameters(void) {

	memset((void*) &Config, 0, sizeof(Config));

	Config.PWMouts.PWM1_2freq = 1;
	Config.PWMouts.PWM3_4freq = 1;
	Config.PWMouts.PWM5_6freq = 1;
	Config.PWMouts.PWM1out = Func_LowBeam;
	Config.PWMouts.PWM2out = Func_HighBeam;
	Config.PWMouts.PWM3out = Func_Brake;
	Config.PWMouts.PWM4out = Func_Horn;
	Config.PWMouts.PWM5out = Func_TurnLeft;
	Config.PWMouts.PWM6out = Func_TurnRight;

	Config.Func.Beam.BeamsMode = Beams_Separate;
	Config.Func.Beam.HighBeamButton = BtExt_8;
	Config.Func.Beam.LowBeamButton = BtEnabled;
	Config.Func.Beam.HighDuty = 100;
	Config.Func.Beam.LowDuty = 100;
	Config.Func.Beam.MinDuty = 20;

	Config.Func.Reverse.HighDuty = 100;
	Config.Func.Reverse.LowDuty = 0;

	Config.Func.Brake.LowBrakeDuty = 20;
	Config.Func.Brake.HighBrakeDuty = 100;
	Config.Func.Brake.StrobePeriod = 5;
	Config.Func.Brake.StrobeCount = 3;

	Config.Func.Horn.HornButton = BtExt_11;

	Config.Func.Turns.LeftButton = BtExt_9;
	Config.Func.Turns.RightButton = BtExt_10;
	Config.Func.Turns.HighDuty = 100;
	Config.Func.Turns.LowDuty = 0;
	Config.Func.Turns.OnTime = 4; //400ms
	Config.Func.Turns.OffTime = 4; //400ms

	Config.Func.Signal.DutyOn1 = 100;
	Config.Func.Signal.DutyOn2 = 100;
	Config.Func.Signal.DutyOn3 = 100;
	Config.Func.Signal.DutyOn4 = 100;

	Config.InputsCfg.InputFilter = 2;
	Config.InputsCfg.BrakeMax = 4000;
	Config.InputsCfg.BrakeMin = 1000;
	Config.InputsCfg.ThrottleMax = 4000;
	Config.InputsCfg.ThrottleMin = 1000;
	Config.InputsCfg.SendControl = 0;
	Config.InputsCfg.SendPorts = 0;
	Config.InputsCfg.T1_Threshold = 50;
	Config.InputsCfg.T2_Threshold = 50;
	//Config.InputsCfg.Brightness = Brightness_BeamHigh;

	for (int i = 0; i < TsensFunc_MAX; i++) {
		Config.Func.FanConrol.Tmin[i] = 40;
		Config.Func.FanConrol.Tmax[i] = 60;
		Config.Func.FanConrol.OutMin[i] = 0;
		Config.Func.FanConrol.OutMax[i] = 100;
	}
}

void LoadStorage(void) {
	Storage_Init(Storagedata, Struct_number, FLASH_ADDRES_BANK0, FLASH_ADDRES_BANK1);

	for (uint8_t i = 0; i < Struct_number; i++) {
		Storage_LoadData(i);
	}
}

void ExportConfig(int index) {
	static char fname[20];

	LC_FileResult_t res = 1;
	int attpt = 0;
	int findex = 0;

	fname[0] = 0;
	snprintf(fname, sizeof(fname), "NuL%d.cfg", index);

	while (res && attpt < 10 && findex < 1000) {
		res = LC_FileOpen(LevcanNodePtr, fname, LC_FA_CreateAlways | LC_FA_Write | LC_FA_Read, LC_Broadcast_Address);

		if (res == LC_FR_TooManyOpenFiles)
			LC_FileClose(LevcanNodePtr, LC_Broadcast_Address);

		attpt++;
	}

	if (res == LC_FR_Ok) {
		res = LC_FilePrintf(LevcanNodePtr, "# %s\n", LevcanNodePtr->DeviceName);
		res = LC_FilePrintf(LevcanNodePtr, "# Network ID: %d\n", LevcanNodePtr->ShortName.NodeID);
	}
	//skip root and autosetup
	for (int dir = Dir_About; dir < LevcanNodePtr->DirectoriesSize && res == LC_FR_Ok; dir++) {
		LCPS_Directory_t *directory = &((LCPS_Directory_t*) LevcanNodePtr->Directories)[dir];
		//	continue;
		if (directory->Entries == NULL || LevcanNodePtr->AccessLevel < directory->AccessLvl)
			continue;

		int comment_out = 0;

		res = LC_FilePrintf(LevcanNodePtr, "\n[%s]\n", directory->Name);

		for (int i = 0; i < directory->Size && res == LC_FR_Ok; i++) {
			//do not export serial
			if (dir == Dir_About && i >= directory->Size - 2)
				continue;

			rdata[0] = 0;
			const LCPS_Entry_t *param = &directory->Entries[i];

			//labels are readonly too
			if (param->Mode & LCP_ReadOnly)
				comment_out = 1;
			else
				comment_out = 0;

			if (param->EntryType > LCP_Folder) {
				LCP_PrintParam(rdata, directory, i);

				if (comment_out)
					res = LC_FilePrintf(LevcanNodePtr, "# %s", rdata);
				else
					res = LC_FilePrintf(LevcanNodePtr, "%s", rdata);
			}
		}
	}
	LC_FilePrintFlush(LevcanNodePtr);

	LC_FileClose(LevcanNodePtr, LC_Broadcast_Address);

	if (res == LC_FR_Ok)
		LC_EventSend(LevcanNodePtr, fname, LANG("Configuration saved!", "Конфигурация сохранена!"), LC_EB_Ok, LC_Broadcast_Address);
	else
		LC_EventSend(LevcanNodePtr, LANG("Error occurred during saving.", "Ошибка во время сохранения."), LANG("Configuration not saved.", "Не сохранено."), LC_EB_Ok,
				LC_Broadcast_Address);

}

void ImportConfig(int index) {
	static char fname[20];

	if (index < 1 || index > 9)
		return;

	LC_FileResult_t res = 1;
	int attpt = 0;
	fname[0] = 0;

	snprintf(fname, sizeof(fname), "NuL%d.cfg", index);

	while (res && attpt < 10) {
		res = LC_FileOpen(LevcanNodePtr, fname, LC_FA_OpenExisting | LC_FA_Read, LC_Broadcast_Address);

		if (res == LC_FR_TooManyOpenFiles)
			LC_FileClose(LevcanNodePtr, LC_Broadcast_Address);
		attpt++;
	}
	if (attpt > 9)
		return;

	LC_FileLseek(LevcanNodePtr, 0);

	const uint32_t read = 128 * 3;
	uint32_t tbr = 0;
	uint32_t br = read;
	uint32_t active_data = 0;

	int16_t dir = -1;
	int16_t idx = -1;
	int16_t param_updated = 0, param_err = 0;
	//display file header
	uint8_t header_catch = 1;
	static char header[24];

	for (; br == read && res == 0;) {
		res = LC_FileRead(LevcanNodePtr, rdata + active_data, read, &br);
		//null terminator
		rdata[br + active_data] = 0;
		active_data += br;

		tbr += br;
		char *rdatpointer = rdata;
		char *rdatpointer2 = rdata;
		if (header_catch && (br < read || active_data > 120) && active_data > 0) {
			//skip blank
			const char *line = rdatpointer;
			for (; isspace((uint8_t ) *line); line++)
				;
			uint linelength = strcspn(line, "\n\r");
			//limit size
			if (linelength > sizeof(header) - 1)
				linelength = sizeof(header) - 1;
			//copy
			strncpy(header, line, linelength);
			header[linelength] = 0;
			header_catch = 0;
		}

		for (; (br < read || active_data > 120) && active_data > 0 && (rdatpointer && *rdatpointer != 0);) {

			rdatpointer = (char*) LCP_ParseParameterName(LevcanNodePtr, rdatpointer, &dir, &idx);
			if (dir >= 0 && idx >= 0) {
				char *rdatout;
				LC_Return_t pres = LC_AccessError;
				if (LevcanNodePtr->AccessLevel >= PD_Directories[dir].Entries[idx].AccessLvl) {
					pres = LCP_ParseParameterValue(&PD_Directories[dir].Entries[idx], PD_Directories[dir].ArrayIndex, rdatpointer, &rdatout);
					rdatpointer = rdatout;
				} else {
					//skip inaccessible parameters
					rdatpointer = strchr(rdatpointer, '\n');
				}

				if (pres == LC_Ok || pres == LC_OutOfRange) {
					param_updated++;
				} else {
					param_err++;
				}
			}
			//todo fix levcan
			if (rdatpointer == 0) {
				active_data = 0;
				break;
			}
			active_data -= rdatpointer - rdatpointer2;
			rdatpointer2 = rdatpointer;
		}
		if (active_data)
			memcpy(rdata, rdatpointer, active_data);
	}

	// Print to the local buffer
	rdata[0] = 0;
	snprintf(rdata, sizeof(rdata), LANG("%s\n%d parameters updated.\n%d errors.", "%s\n%d параметров обновлено.\n%d ошибок."), header, param_updated, param_err);
	LC_EventSend(LevcanNodePtr, rdata, LANG("Configuration loaded!", "Конфигурация загружена!"), LC_EB_Ok, LC_Broadcast_Address);
	LC_FileClose(LevcanNodePtr, LC_Broadcast_Address);
}
