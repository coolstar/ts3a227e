#if !defined(_TS3A227E_H_)
#define _TS3A227E_H_

#pragma warning(disable:4200)  // suppress nameless struct/union warning
#pragma warning(disable:4201)  // suppress nameless struct/union warning
#pragma warning(disable:4214)  // suppress bit field types other than int warning
#include <initguid.h>
#include <wdm.h>

#pragma warning(default:4200)
#pragma warning(default:4201)
#pragma warning(default:4214)
#include <wdf.h>

#pragma warning(disable:4201)  // suppress nameless struct/union warning
#pragma warning(disable:4214)  // suppress bit field types other than int warning
#include <hidport.h>

#include "hidcommon.h"
#include "spb.h"

//
// String definitions
//

#define DRIVERNAME                 "ts3a227e.sys: "

#define TS3A227E_POOL_TAG            (ULONG) 'a3sT'

	typedef UCHAR HID_REPORT_DESCRIPTOR, *PHID_REPORT_DESCRIPTOR;

#ifdef DESCRIPTOR_DEF
HID_REPORT_DESCRIPTOR DefaultReportDescriptor[] = {
	//
	// Consumer Control starts here
	//
	0x05, 0x0C, /*		Usage Page (Consumer Devices)		*/
	0x09, 0x01, /*		Usage (Consumer Control)			*/
	0xA1, 0x01, /*		Collection (Application)			*/
	0x85, REPORTID_MEDIA,	/*		Report ID=1							*/
	0x05, 0x0C, /*		Usage Page (Consumer Devices)		*/
	0x15, 0x00, /*		Logical Minimum (0)					*/
	0x25, 0x01, /*		Logical Maximum (1)					*/
	0x75, 0x01, /*		Report Size (1)						*/
	0x95, 0x04, /*		Report Count (4)					*/
	0x09, 0xCD, /*		Usage (Play / Pause)				*/
	0x09, 0xCF, /*		Usage (Voice Command)				*/
	0x09, 0xE9, /*		Usage (Volume Up)					*/
	0x09, 0xEA, /*		Usage (Volume Down)					*/
	0x81, 0x02, /*		Input (Data, Variable, Absolute)	*/
	0x95, 0x04, /*		Report Count (4)					*/
	0x81, 0x01, /*		Input (Constant)					*/
	0xC0,        /*        End Collection                        */
};


//
// This is the default HID descriptor returned by the mini driver
// in response to IOCTL_HID_GET_DEVICE_DESCRIPTOR. The size
// of report descriptor is currently the size of DefaultReportDescriptor.
//

CONST HID_DESCRIPTOR DefaultHidDescriptor = {
	0x09,   // length of HID descriptor
	0x21,   // descriptor type == HID  0x21
	0x0100, // hid spec release
	0x00,   // country code == Not Specified
	0x01,   // number of HID class descriptors
	{ 0x22,   // descriptor type 
	sizeof(DefaultReportDescriptor) }  // total length of report descriptor
};
#endif

#define true 1
#define false 0

typedef struct _TS3A227E_CONTEXT
{

	WDFDEVICE FxDevice;

	WDFQUEUE ReportQueue;

	BYTE DeviceMode;

	SPB_CONTEXT I2CContext;

	WDFINTERRUPT Interrupt;

	BOOLEAN ConnectInterrupt;

	BOOLEAN RegsSet;

	bool plugged;
	bool mic_present;
	unsigned int buttons_held;

} TS3A227E_CONTEXT, *PTS3A227E_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(TS3A227E_CONTEXT, GetDeviceContext)

//
// Function definitions
//

extern "C" {
	DRIVER_INITIALIZE DriverEntry;
}

EVT_WDF_DRIVER_UNLOAD Ts3a227eDriverUnload;

EVT_WDF_DRIVER_DEVICE_ADD Ts3a227eEvtDeviceAdd;

EVT_WDFDEVICE_WDM_IRP_PREPROCESS Ts3a227eEvtWdmPreprocessMnQueryId;

EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL Ts3a227eEvtInternalDeviceControl;

NTSTATUS
Ts3a227eGetHidDescriptor(
	IN WDFDEVICE Device,
	IN WDFREQUEST Request
);

NTSTATUS
Ts3a227eGetReportDescriptor(
	IN WDFDEVICE Device,
	IN WDFREQUEST Request
);

NTSTATUS
Ts3a227eGetDeviceAttributes(
	IN WDFREQUEST Request
);

NTSTATUS
Ts3a227eGetString(
	IN WDFREQUEST Request
);

NTSTATUS
Ts3a227eWriteReport(
	IN PTS3A227E_CONTEXT DevContext,
	IN WDFREQUEST Request
);

NTSTATUS
Ts3a227eProcessVendorReport(
	IN PTS3A227E_CONTEXT DevContext,
	IN PVOID ReportBuffer,
	IN ULONG ReportBufferLen,
	OUT size_t* BytesWritten
);

NTSTATUS
Ts3a227eReadReport(
	IN PTS3A227E_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
);

NTSTATUS
Ts3a227eSetFeature(
	IN PTS3A227E_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
);

NTSTATUS
Ts3a227eGetFeature(
	IN PTS3A227E_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
);

PCHAR
DbgHidInternalIoctlString(
	IN ULONG        IoControlCode
);

//
// Helper macros
//

#define DEBUG_LEVEL_ERROR   1
#define DEBUG_LEVEL_INFO    2
#define DEBUG_LEVEL_VERBOSE 3

#define DBG_INIT  1
#define DBG_PNP   2
#define DBG_IOCTL 4

#if 0
#define Ts3a227ePrint(dbglevel, dbgcatagory, fmt, ...) {          \
    if (Ts3a227eDebugLevel >= dbglevel &&                         \
        (Ts3a227eDebugCatagories && dbgcatagory))                 \
	    {                                                           \
        DbgPrint(DRIVERNAME);                                   \
        DbgPrint(fmt, __VA_ARGS__);                             \
	    }                                                           \
}
#else
#define Ts3a227ePrint(dbglevel, fmt, ...) {                       \
}
#endif

#endif