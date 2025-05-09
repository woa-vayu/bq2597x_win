/*++
Copyright (c) Microsoft Corporation. All Rights Reserved.
Sample code. Dealpoint ID #843729.

Module Name:

spb.h

Abstract:

This module contains the touch driver I2C helper definitions.

Environment:

Kernel Mode

Revision History:

--*/

#pragma once

#include <wdm.h>
#include <wdf.h>

#define DEFAULT_SPB_BUFFER_SIZE 64
#define RESHUB_USE_HELPER_ROUTINES

//
// SPB (I2C) context
//

typedef struct _SPB_CONTEXT
{
	WDFIOTARGET SpbIoTarget;
	LARGE_INTEGER I2cResHubId;
	WDFMEMORY WriteMemory;
	WDFMEMORY ReadMemory;
	WDFWAITLOCK SpbLock;
} SPB_CONTEXT;

NTSTATUS
SpbWriteRead(
	_In_                            SPB_CONTEXT* SpbContext,
	_In_reads_(SendLength)          PVOID           SendData,
	_In_                            USHORT          SendLength,
	_Out_writes_(Length)            PVOID           Data,
	_In_                            USHORT          Length,
	_In_                            ULONG           DelayUs
);

NTSTATUS
SpbXferDataSynchronously(
	_In_ SPB_CONTEXT* SpbContext,
	_In_ PVOID SendData,
	_In_ ULONG SendLength,
	_In_reads_bytes_(Length) PVOID Data,
	_In_ ULONG Length
);

VOID
SpbTargetDeinitialize(
	IN WDFDEVICE FxDevice,
	IN SPB_CONTEXT* SpbContext
);

NTSTATUS
SpbTargetInitialize(
	IN WDFDEVICE FxDevice,
	IN SPB_CONTEXT* SpbContext
);

NTSTATUS
SpbWriteDataSynchronously(
	_In_ SPB_CONTEXT* SpbContext,
	_In_ PVOID Data,
	_In_ ULONG Length
);

NTSTATUS
SpbWriteDataSynchronouslyEx(
	_In_ SPB_CONTEXT* SpbContext,
	_In_ PVOID Data,
	_In_ ULONG Length,
	_In_ PVOID Data2,
	_In_ ULONG Length2
);