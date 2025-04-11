#include <trace.h>

#pragma warning(disable:4200)  // suppress nameless struct/union warning
#pragma warning(disable:4201)  // suppress nameless struct/union warning
#pragma warning(disable:4214)  // suppress bit field types other than int warning
#include <initguid.h>
#include <wdm.h>

#pragma warning(default:4200)
#pragma warning(default:4201)
#pragma warning(default:4214)
#include <wdf.h>

#include <acpiioct.h>
#include <ntstrsafe.h>
#include <spb.h>
#include <spb1.h>

//
// String definitions
//

#define BQ2597X_POOL_TAG            (ULONG) '7495'

#define bool int

#define true 1
#define false 0

typedef struct _BQ2597X_CONTEXT
{

	WDFDEVICE FxDevice;

	WDFINTERRUPT InterruptObject;

	WDFQUEUE ReportQueue;

	SPB_CONTEXT I2CContext;

	BOOLEAN DevicePoweredOn;

	WDFWAITLOCK DataLock;
	
	int part_no;
	int revision;

	int mode;
	
	bool volt_qual;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool vbus_present;

	bool usb_present;
	bool charge_enabled;	/* Register bit status */

	/* ADC reading */
	int vbat_volt;
	int vbus_volt;
	int vout_volt;
	int vac_volt;

	int ibat_curr;
	int ibus_curr;

	int bat_temp;
	int bus_temp;
	int die_temp;

	/* alarm/fault status */
	bool bat_ovp_fault;
	bool bat_ocp_fault;
	bool bus_ovp_fault;
	bool bus_ocp_fault;

	bool bat_ovp_alarm;
	bool bat_ocp_alarm;
	bool bus_ovp_alarm;
	bool bus_ocp_alarm;

	bool bat_ucp_alarm;

	bool bat_therm_alarm;
	bool bus_therm_alarm;
	bool die_therm_alarm;

	bool bat_therm_fault;
	bool bus_therm_fault;
	bool die_therm_fault;

	bool therm_shutdown_flag;
	bool therm_shutdown_stat;

	bool vbat_reg;
	bool ibat_reg;

	int  prev_alarm;
	int  prev_fault;

	int chg_ma;
	int chg_mv;

	int charge_state;

	int skip_writes;
	int skip_reads;

	struct power_supply* fc2_psy;

} BQ2597X_CONTEXT, *PBQ2597X_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(BQ2597X_CONTEXT, GetDeviceContext)

//
// Function definitions
//

DRIVER_INITIALIZE DriverEntry;

EVT_WDF_DRIVER_UNLOAD Bq2597xDriverUnload;

EVT_WDF_DRIVER_DEVICE_ADD Bq2597xEvtDeviceAdd;

EVT_WDFDEVICE_WDM_IRP_PREPROCESS Bq2597xEvtWdmPreprocessMnQueryId;

EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL Bq2597xEvtInternalDeviceControl;