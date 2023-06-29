#include "levcan.h"
#include "levcan_objects.h"
#pragma once

typedef struct {
	LC_Obj_Temperature_t Temp;
	LC_Obj_Supply_t Supply;
	LC_Obj_ActiveFunctions_t Functions;
} CANData_t;

extern CANData_t CANdata;
extern LC_NodeDescriptor_t *LevcanNodePtr;
void Network_Update(uint32_t tick);
void Network_Init(uint8_t node_id);
