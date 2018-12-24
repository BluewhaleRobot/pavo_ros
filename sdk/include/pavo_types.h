#pragma once

#include "sys_types.h"


#if defined(_WIN32)
#pragma pack(1)
#define __DATA_ALIGN__  
#else
#define __DATA_ALIGN__ __attribute__((packed))
#endif

typedef struct pavo_response_scan
{
	uint16_t angle;
	uint16_t distance;
	uint8_t  intensity;
} __DATA_ALIGN__ pavo_response_scan_t;

typedef struct pavo_response_pcd
{
	double x;
	double y;
	double z;  //fixed as 0 for single-laser device
	uint8_t  intensity;
} __DATA_ALIGN__ pavo_response_pcd_t;


#if defined(_WIN32)
#pragma pack()
#endif




