/* 
 * SOEM EtherCAT exmaple
 * Ported to raspberry pi by Ho Tam - thanhtam.h[at]gmail.com 
 */
 

#ifndef _PDO_DEF_
#define _PDO_DEF_

#include "servo_def.h"

//EPOS4 PDO mapping

typedef union _mapping_obj{
	uint32_t obj;
	struct {
		uint16_t index;
		uint8_t subindex;
		uint8_t size;
	};
} mapping_obj;

//0x1600 RxPDO
typedef struct PACKED
{
	UINT16	ControlWord;		//0x6040
	INT32	TargetPosition;		//0x607A
	INT32	TargetVelocity;		//0x60FF
	//INT32	PositionOffset;		//0x60B0
	//INT32	VelocityOffset;		//0x60B1
	//INT16	TorqueOffset;		//0x60B2
	//INT8    HomingMethod;       //0x6098
}EPOS4_DRIVE_RxPDO_t;

//0x1A00 TxPDO
typedef struct PACKED
{
	UINT16	StatusWord;			//0x6041
	INT32	PositionActualValue;		//0x6064
	INT32	VelocityActualValue;		//0x606C
	//INT16	TorqueActualValue;			//0x6077
	UINT32	DigitalInput;				//0x60FD
        UINT16	ErrorCode;                  //0x603F
}EPOS4_DRIVE_TxPDO_t;


typedef struct _EPOS4_ServoDrive
{
	EPOS4_DRIVE_RxPDO_t 	OutParam;
	EPOS4_DRIVE_TxPDO_t 	InParam;
} EPOS4_ServoDrive_t;

typedef struct _EPOS4_Drive_pt
{
	EPOS4_DRIVE_RxPDO_t 	*ptOutParam;
	EPOS4_DRIVE_TxPDO_t 	*ptInParam;
} EPOS4_Drive_pt;


#endif //_PDO_DEF_
