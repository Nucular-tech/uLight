/*
 * network.c
 *
 *  Created on: 31 jan 2020
 *      Author: User
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "armutils.h"
#include "Resconfig.h"
#include "Data/parameters.h"
#include "Data/hwconfig.h"

#include "can_hal.h"
#include "levcan.h"
#include "levcan_objects.h"
#include "levcan_paramserver.h"
#include "levcan_fileclient.h"
#include "levcan_events.h"
#include "logic.h"

extern void proceedSWU(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);

extern const LCPS_Directory_t PD_Directories[];
extern const uint32_t PD_Directories_size;

void networkShutdown(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);

const LC_Object_t light_node_obj[] = { //
		{ LC_SYS_Shutdown, { .Readable = 1, .Writable = 1, .Function = 1 }, 0, &networkShutdown },		//
		{ LC_SYS_SWUpdate, { .Writable = 1, .Function = 1 }, 4, &proceedSWU }, //
		{ LC_Obj_ActiveFunctions, { .Writable = 1, .Function = 1 }, sizeof(LC_Obj_ActiveFunctions_t), &LogicProcessData }, //
		{ LC_Obj_Temperature, { .Writable = 1, .Function = 1 }, sizeof(LC_Obj_Temperature_t), &LogicProcessData }, //
		{ LC_Obj_Buttons, { .Writable = 1, .Function = 1 }, sizeof(LC_Obj_Buttons_t), &LogicProcessData }, //
		};
const uint16_t light_node_obj_size = sizeof(light_node_obj) / sizeof(light_node_obj[0]);

const LC_DriverCalls_t nodeDrv = { LC_HAL_Send, LC_HAL_CreateFilterMasks, LC_HAL_TxHalfFull };

LC_NodeDescriptor_t LevcanNode;
LC_NodeDescriptor_t *LevcanNodePtr;

extern const Version_t VersionControl;
extern LCPS_Entry_t PD_About[];
extern void delay_ms(uint32_t delay);

void Network_Init(uint8_t node_id) {

	CAN_Init(0x01050002);

	LC_NodeDescriptor_t *node_init = &LevcanNode;
	LC_InitNodeDescriptor(node_init);
	node_init->Driver = &nodeDrv;

	LC_FileClientInit(node_init);
	LCP_ParameterServerInit(node_init);
	LC_EventInit(node_init);

	node_init->DeviceName = "Nucular uLight";
	node_init->NodeName = "Nucular uLight";
	node_init->VendorName = "Nucular.Tech";
	node_init->Serial[0] = STM_UID.UID_0;
	node_init->Serial[1] = STM_UID.UID_1;
	node_init->Serial[2] = STM_UID.UID_2;
	node_init->Serial[3] = 0x4e55434c;

	node_init->Objects = (LC_Object_t*) light_node_obj;
	node_init->ObjectsSize = light_node_obj_size;
	node_init->Directories = (void*) PD_Directories;
	node_init->DirectoriesSize = PD_Directories_size;

	node_init->ShortName.ManufacturerCode = 0x2D2;
	node_init->ShortName.DeviceType = LC_Device_Light;
	node_init->ShortName.NodeID = node_id;
	node_init->ShortName.SWUpdates = 1;
	node_init->ShortName.Configurable = 1;
	node_init->ShortName.CodePage = 1251;

	LC_CreateNode(node_init);

	//PD_Root[0].Name = HWConfig.Name;
	PD_About[0].Name = HWConfig.Name;
	PD_About[1].TextData = (void*) &VersionControl.Date;
	PD_About[2].TextData = (void*) &VersionControl.Version;
	PD_About[3].TextData = HWConfig.Date;
	PD_About[4].TextData = HWConfig.Version;

	LevcanNodePtr = &LevcanNode;
	LEVCAN_Node_Drv = &LevcanNode;
}

void Network_Update(uint32_t tick) {
	static int nm = 0;
	if (CAN_ERR) {
		delay_ms(100);
		CAN_Start();
		CAN_ERR = 0;
	}
	nm++;
	LC_ReceiveManager(&LevcanNode);
	//manager should work quickly to free levcan message objects
	LC_NetworkManager(&LevcanNode, tick );
}

void networkShutdown(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size) {
	(void) size;
	(void) node;
	(void) header;
	(void) data;
	NVIC_SystemReset();
}
