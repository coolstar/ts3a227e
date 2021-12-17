#if !defined(_TS3A227E_COMMON_H_)
#define _TS3A227E_COMMON_H_

//
//These are the device attributes returned by vmulti in response
// to IOCTL_HID_GET_DEVICE_ATTRIBUTES.
//

#define TS3A227E_PID              0xBACC
#define TS3A227E_VID              0x00FF
#define TS3A227E_VERSION          0x0001

//
// These are the report ids
//

#define REPORTID_MEDIA	0x01

#pragma pack(1)
typedef struct _TS3A227E_MEDIA_REPORT
{

	BYTE      ReportID;

	BYTE	  ControlCode;

} Ts3a337eMediaReport;
#pragma pack()

//
// Feature report infomation
//

#define DEVICE_MODE_MOUSE        0x00
#define DEVICE_MODE_SINGLE_INPUT 0x01
#define DEVICE_MODE_MULTI_INPUT  0x02

#endif
