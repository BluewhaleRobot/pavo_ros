#pragma once

#include "sys_types.h"
#include "pavo_types.h"

#define PAVO_CMD_LEN (4)
#define PAVO_REG_LEN (4)
#define PAVO_RESPONSE_LEN (4)
#define DATA_PER_FIRING (2)
#define FIRING_PER_PKT (12)

#define MAX_RESPONSE_PER_FRAME (4096)

#define NUM_LASERS (1)

#define DEGREE_SHIFT_MIN (10)
#define DEGREE_SHIFT_MAX (35999)

#define DEGREE_SCOPE_MIN (0)
#define DEGREE_SCOPE_MAX (35999)

#define PAVO_CONN_DETECT_INTERVAL (100) //ms

enum PAVO_REG_INDEX {
	REG_DEVICE_TYPE_GET = 0,       
	REG_DEVICE_CONNECTION_GET,
	REG_DATA_EN_GET,      
	REG_DATA_EN_SET,
	
	REG_DEST_IP_GET,  //4
	REG_DEVICE_IP_GET,
	REG_DEST_PORT_GET, 
	REG_DEVICE_PORT_GET,
	
	REG_MOTOR_SPEED_GET, //8
	
	REG_DEST_IP_CONFIG_GET, //9
	REG_DEST_IP_CONFIG_SET,
	REG_DEVICE_IP_CONFIG_GET,
	REG_DEVICE_IP_CONFIG_SET,
	
	REG_DEST_PORT_CONFIG_GET, //13
	REG_DEST_PORT_CONFIG_SET,
	REG_DEVICE_PORT_CONFIG_GET,
	REG_DEVICE_PORT_CONFIG_SET,
	
	REG_MOTOR_CONFIG_GET, //17
	REG_MOTOR_CONFIG_SET,
	REG_POINTCLOUD_MERGE_GET,
	REG_POINTCLOUD_MERGE_SET,
	
	REG_NETCONFIG_APPLY, //21
	REG_DEGREE_SHIFT_GET,
	REG_DEGREE_SHIFT_SET,
	REG_MOTOR_EN_GET,
	REG_MOTOR_EN_SET,
	
	REG_DEVICE_PN_GET,  //26
	REG_DEVICE_SN_GET,
	
	REG_DATA_TRANSFER_MODE,
};

enum PAVO_CMD_INDEX {      
	CMD_DATA_EN_SET_ENABLE = 0,
	CMD_DATA_EN_SET_DISABLE,
	
	CMD_POINTCLOUD_MERGE_NONE, //2
	CMD_POINTCLOUD_MERGE_2,
	CMD_POINTCLOUD_MERGE_4,
	CMD_POINTCLOUD_MERGE_8,
	CMD_POINTCLOUD_MERGE_16,
	
	CMD_NETCONFIG_APPLY, //7
	CMD_MOTOR_EN_SET_ENABLE,
	CMD_MOTOR_EN_SET_DISABLE
};


static const unsigned char PAVO_REG[][PAVO_REG_LEN]  = {
	{0xEB, 0x90, 0xB0, 0xF0},  //DEVICE_TYPE_GET	//0       
	{0xEB, 0x90, 0xB0, 0xF1},  //DEVICE_CONNECTION_GET   
	{0xEB, 0x90, 0xB0, 0xA0},  //DATA_EN_GET             
	{0xEB, 0x90, 0xA0, 0xA0},  //DATA_EN_SET  
	
	{0xEB, 0x90, 0xB0, 0x00},  //REG_DEST_IP_GET	//4
	{0xEB, 0x90, 0xB0, 0x01},  //REG_DEVICE_IP_GET 
	{0xEB, 0x90, 0xB0, 0x02},  //REG_DEST_PORT_GET 
	{0xEB, 0x90, 0xB0, 0x03},  //REG_DEVICE_PORT_GET 
	
	{0xEB, 0x90, 0xB0, 0xD0},  //REG_MOTOR_SPEED_GET	//8
	
	{0xEB, 0x90, 0xB0, 0xE0},  //REG_DEST_IP_CONFIG_GET	//9
	{0xEB, 0x90, 0xA0, 0xE0},  //REG_DEST_IP_CONFIG_SET 
	{0xEB, 0x90, 0xB0, 0xE1},  //REG_DEVICE_IP_CONFIG_GET
	{0xEB, 0x90, 0xA0, 0xE1},  //REG_DEVICE_IP_CONFIG_SET
	
	{0xEB, 0x90, 0xB0, 0xE2},  //REG_DEST_PORT_CONFIG_GET	//13
	{0xEB, 0x90, 0xA0, 0xE2},  //REG_DEST_PORT_CONFIG_SET 
	{0xEB, 0x90, 0xB0, 0xE3},  //REG_DEVICE_PORT_CONFIG_GET
	{0xEB, 0x90, 0xA0, 0xE3},  //REG_DEVICE_PORT_CONFIG_SET 
	
	{0xEB, 0x90, 0xB0, 0x10},  //REG_MOTOR_CONFIG_GET //17
	{0xEB, 0x90, 0xA0, 0x10},  //REG_MOTOR_CONFIG_SET
	{0xEB, 0x90, 0xB0, 0x11},  //REG_POINTCLOUD_MERGE_GET
	{0xEB, 0x90, 0xA0, 0x11},  //REG_POINTCLOUD_MERGE_SET
	
	{0xEB, 0x90, 0xA0, 0xEF},  //REG_NETCONFIG_APPY //21
	{0xEB, 0x90, 0xB0, 0x13},  //REG_DEGREE_SHIFT_GET
	{0xEB, 0x90, 0xA0, 0x13},  //REG_DEGREE_SHIFT_SET
	{0xEB, 0x90, 0xB0, 0xA5},  //REG_MOTOR_EN_GET,
	{0xEB, 0x90, 0xA0, 0xA5},  //REG_MOTOR_EN_SET
	
	{0xEB, 0x90, 0xB0, 0x78},  //REG_DEVICE_PN_GET,
	{0xEB, 0x90, 0xB0, 0x79},   //REG_DEVICE_SN_GET

	{0xEB, 0x90, 0xC0, 0x00 }   //REG_DATA_TRANSFER_DATA_MODE
	
};

static const unsigned char PAVO_CMD[][PAVO_CMD_LEN]  = {
	{0x00, 0x00, 0x00, 0x01},   //CMD_DATA_EN_SET_ENABLE   
	{0x00, 0x00, 0x00, 0x00},   //CMD_DATA_EN_SET_DISABLE 
	
	{0x00, 0x00, 0x00, 0x01},   //CMD_POINTCLOUD_MERGE_NONE //2
	{0x00, 0x00, 0x00, 0x02},   //CMD_POINTCLOUD_MERGE_2
	{0x00, 0x00, 0x00, 0x04},   //CMD_POINTCLOUD_MERGE_4
	{0x00, 0x00, 0x00, 0x08},   //CMD_POINTCLOUD_MERGE_8
	{0x00, 0x00, 0x00, 0x10},   //CMD_POINTCLOUD_MERGE_16
	
	{0x00, 0x00, 0x00, 0x01},	//CMD_NETCONFIG_APPLY
	{0x00, 0x00, 0x00, 0x01},   //CMD_MOTOR_EN_SET_ENABLE,
	{0x00, 0x00, 0x00, 0x00}	//CMD_MOTOR_EN_SET_DISABLE
};

static const  char PAVO_RESPONSE_CONNECTED[] = {0x5a, 0x5a, 0x5a, 0x5a};
static const unsigned int VALID_MOTOR_SPEED[] = {10, 15, 20, 25, 30};
static const unsigned int VALID_POINT_CLOUD_MERGE_COEF[] = {1, 2, 4, 8, 16};


#if defined(_WIN32)
#pragma pack(1)
#define __DATA_ALIGN__  
#else
#define __DATA_ALIGN__ __attribute__((packed))
#endif

typedef struct pavo_laser_return
{
  uint16_t distance;
  uint8_t intensity;
} __DATA_ALIGN__ pavo_laser_return_t;


typedef struct pavo_firing_data
{
  uint16_t blockIdentifier;
  uint16_t rotationalPosition;
  pavo_laser_return_t laserReturns[DATA_PER_FIRING];
} __DATA_ALIGN__ pavo_firing_data_t;

typedef struct pavo_data_packet
{
  pavo_firing_data_t firingData[FIRING_PER_PKT];
  uint32_t timestamp;
  uint8_t blank1;
  uint8_t blank2;
}__DATA_ALIGN__ pavo_data_packet_t;


typedef struct pavo_cmd_packet_single {
    uint8_t reg[PAVO_REG_LEN];
} __DATA_ALIGN__ pavo_cmd_packet_single_t;

typedef struct pavo_cmd_packet_dual {
    uint8_t reg[PAVO_REG_LEN];
	uint8_t cmd[PAVO_CMD_LEN];
} __DATA_ALIGN__ pavo_cmd_packet_dual_t;


typedef struct pavo_response_packet {
    uint8_t cmd[PAVO_CMD_LEN];
	uint8_t cmd_response[PAVO_RESPONSE_LEN];
} __DATA_ALIGN__ pavo_response_packet_t;


#if defined(_WIN32)
#pragma pack()
#endif




inline pavo_response_scan_t* GenWedgeData()
{
	pavo_response_scan_t* wedge = new pavo_response_scan_t();
	wedge->angle = 0xFFFF;
	wedge->distance = 0xFFFF;
	wedge->intensity = 0xFF;
	return wedge;
}

inline bool IsWedgeData(pavo_response_scan_t* wedge)
{
	return (wedge->angle == 0xFFFF) && (wedge->distance == 0xFFFF) && (wedge->intensity=0xFF);
}

inline bool IsCMDResponse(unsigned char* data_ptr)
{
	return (*data_ptr == 0xEB) && (*(data_ptr+1) == 0x90) ;
}

inline bool IsValidMotorSpeed(int speed)
{
	int i;
	int size = sizeof(VALID_MOTOR_SPEED)/sizeof(int);
	for(i=0;i<size; i++)
	{
		if(speed == VALID_MOTOR_SPEED[i])
			return true;
	}
	return false;
}

inline bool IsValidPointCloudMergeCoef(int coef)
{
	int i;
	int size = sizeof(VALID_POINT_CLOUD_MERGE_COEF)/sizeof(int);
	for(i=0;i<size; i++)
	{
		if(coef == VALID_POINT_CLOUD_MERGE_COEF[i])
			return true;
	}
	return false;
}


