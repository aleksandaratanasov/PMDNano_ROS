/*****************************************************************************
 * PMDSDK 2
 * 
 * Copyright (c) 2006-2012 PMD Technologies GmbH
 * All Rights Reserved.
 *
 * Author: Martin Profittlich
 *
 *****************************************************************************/
#pragma once
#include <stddef.h>

typedef float          PMDFloat32;
typedef double         PMDFloat64;
typedef PMDFloat32     PMDFloat;
typedef unsigned       PMDUInt32;
typedef signed int     PMDSInt32;
typedef unsigned short PMDUInt16;
typedef signed short   PMDSInt16;
typedef unsigned char  PMDUInt8;
typedef signed char    PMDSInt8;

typedef enum { AtLeast, AtMost, CloseTo } Proximity;

#define PMD_OK                0
#define PMD_RUNTIME_ERROR     1024
#define PMD_GENERIC_ERROR     1025
#define PMD_DISCONNECTED      1026
#define PMD_INVALID_VALUE     1027
#define PMD_TIMEOUT_ERROR     1028
#define PMD_LOGIC_ERROR       2048
#define PMD_UNKNOWN_HANDLE    2049
#define PMD_NOT_IMPLEMENTED   2050
#define PMD_OUT_OF_BOUNDS     2051
#define PMD_RESOURCE_ERROR    4096
#define PMD_FILE_NOT_FOUND    4097
#define PMD_COULD_NOT_OPEN    4098
#define PMD_DATA_NOT_FOUND    4099
#define PMD_END_OF_DATA       4100
#define PMD_DEVICE_IS_BUSY    4101

#ifndef PMD_NO_DEPRECATED

#define PMD_COULD_NOT_CONNECT 4098

#endif

#define PMD_INTERFACE_VERSION_1_0_0 0x00010000
#define PMD_INTERFACE_VERSION_1_1_0 0x00010100
#define PMD_INTERFACE_VERSION_1_1_1 0x00010101
#define PMD_INTERFACE_VERSION_1_2_0 0x00010200
#define PMD_INTERFACE_VERSION_1_2_1 0x00010201
#define PMD_INTERFACE_VERSION_1_3_0 0x00010300
#define PMD_CURRENT_INTERFACE_VERSION PMD_INTERFACE_VERSION_1_3_0
