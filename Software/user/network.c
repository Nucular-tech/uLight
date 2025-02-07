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

#include "network.h"
#include "can_hal.h"
#include "levcan.h"
#include "levcan_objects.h"
#include "levcan_paramserver.h"
#include "levcan_fileclient.h"
#include "levcan_events.h"
#include "logic.h"

extern void proceedSWU(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);

//reference to configurable parameters of this device
extern const LCPS_Directory_t PD_Directories[];
extern const uint32_t PD_Directories_size;
extern LCPS_Entry_t PD_About[];

void networkShutdown(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size);

//User parameters
CANData_t CANdata;

// CAN data objects, accessible over LEVCAN
// Each data object can be readable or writable, or both
// Data objects used to handle receive and automatic send by request by LEVCAN library
// @formatter:off
const LC_Object_t light_node_obj[] = { //
		{ LC_SYS_Shutdown, 			{ .Readable = 1, .Writable = 1, .Function = 1 }, 0, &networkShutdown },	// This object have a callback on read and write, with 0 byte size
		{ LC_SYS_SWUpdate, 			{ .Writable = 1, .Function = 1 }, 4, &proceedSWU }, // This object have a write callback with 4 byte size
		{ LC_Obj_ActiveFunctions, 	{ .Writable = 1, .Function = 1 }, sizeof(LC_Obj_ActiveFunctions_t), &LogicProcessData }, // This object have a write callback with sizeof(LC_Obj_ActiveFunctions_t) byte size
		{ LC_Obj_Temperature, 		{ .Writable = 1, .Function = 1 }, sizeof(LC_Obj_Temperature_t), &LogicProcessData }, //
		{ LC_Obj_Buttons, 			{ .Writable = 1, .Function = 1 }, sizeof(LC_Obj_Buttons_t), &LogicProcessData }, //
		{ LC_Obj_Temperature, 		{ .Readable = 1 }, sizeof(LC_Obj_Temperature_t), 	&CANdata.Temp }, // This object can be readed over network, data size is sizeof(LC_Obj_Temperature_t)
		{ LC_Obj_DCSupply, 			{ .Readable = 1 }, sizeof(LC_Obj_Supply_t), 		&CANdata.Supply }, //
		{ LC_Obj_ActiveFunctions, 	{ .Readable = 1 }, sizeof(LC_Obj_ActiveFunctions_t), &CANdata.Functions }, //
};
// @formatter:on
// Count of data objects
const uint16_t light_node_obj_size = sizeof(light_node_obj) / sizeof(light_node_obj[0]);

// CAN HAL driver functions
const LC_DriverCalls_t nodeDrv = { LC_HAL_Send, LC_HAL_CreateFilterMasks, LC_HAL_TxHalfFull };

// Our LEVCAN node and it's pointer
LC_NodeDescriptor_t LevcanNode;
LC_NodeDescriptor_t *LevcanNodePtr;

extern const Version_t VersionControl;
extern void delay_ms(uint32_t delay);

void Network_Init(uint8_t node_id) {

	//24 Mhz base clock
	//Setup CAN HAL at 1 MBit
	CAN_Init(0x01050002);

	//Initialize LEVCAN node
	LC_NodeDescriptor_t *node_init = &LevcanNode;
	LC_InitNodeDescriptor(node_init);
	//setup CAN HAL driver
	node_init->Driver = &nodeDrv;

	//Setup file client for this node, to access file operations over CAN bus
	LC_FileClientInit(node_init);
	//Setup parameter server for this node, to make it configurable over CAN bus
	LCP_ParameterServerInit(node_init, 0);
	//This node may produce Events, they should be displayed on dashboard
	LC_EventInit(node_init);

	//Setup device name and its serial number
	node_init->DeviceName = "Nucular uLight";
	node_init->NodeName = "Nucular uLight";
	node_init->VendorName = "Nucular.Tech";
	//Serial number used for Node ID setup when there is ID collision
	node_init->Serial[0] = STM_UID.UID_0;
	node_init->Serial[1] = STM_UID.UID_1;
	node_init->Serial[2] = STM_UID.UID_2;
	node_init->Serial[3] = 0x4e55434c;

	//Node data objects
	node_init->Objects = (LC_Object_t*) light_node_obj;
	node_init->ObjectsSize = light_node_obj_size;
	//Configurable parameters of this node
	node_init->Directories = (void*) PD_Directories;
	node_init->DirectoriesSize = PD_Directories_size;

	//ShortName is a structure used in quick identification of node in LEVCAN
	node_init->ShortName.ManufacturerCode = 0x2D2;
	node_init->ShortName.DeviceType = LC_Device_Light; //Type of device, used for user-code logic filters
	node_init->ShortName.NodeID = node_id; //saved ID from previous start
	node_init->ShortName.SWUpdates = 1; //This node can be updated over CAN
	node_init->ShortName.Configurable = 1; //This node have configurable parameters
	node_init->ShortName.CodePage = 1251; //Used codepage of parameters text, in this case Windows-1251

	//Start node
	LC_CreateNode(node_init);

	//Some text data substitution into parameters
	PD_About[0].Name = HWConfig.Name;
	PD_About[1].TextData = (void*) &VersionControl.Date;
	PD_About[2].TextData = (void*) &VersionControl.Version;
	PD_About[3].TextData = HWConfig.Date;
	PD_About[4].TextData = HWConfig.Version;

	//Save node pointers, used in function calls
	LevcanNodePtr = &LevcanNode;
	LEVCAN_Node_Drv = &LevcanNode;
}

//Network update tick, should be between 1-10ms for optimal speed.
//If speed is slower you may need to increase CAN HAL RX message buffer size to avoid message buffer overflow
//Theoretical CAN throughput is about 8 messages every 1ms
void Network_Update(uint32_t tick) {
	static int nm = 0;
	//Restart CAN driver if it is in BUS-OFF state
	if (CAN_ERR) {
		delay_ms(100);
		CAN_Start();
		CAN_ERR = 0;
	}
	nm++;
	//This function processes all incoming messages
	LC_ReceiveManager(&LevcanNode);
	//Manager should work quickly to free LEVCAN message objects
	LC_NetworkManager(&LevcanNode, tick);
}

//Callback function to restart device over LEVCAN
void networkShutdown(LC_NodeDescriptor_t *node, LC_Header_t header, void *data, int32_t size) {
	(void) size;
	(void) node;
	(void) header;
	(void) data;
	NVIC_SystemReset();
}
