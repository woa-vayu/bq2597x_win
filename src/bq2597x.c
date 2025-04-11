#include <bq2597x.h>
#include <registers.h>

#include <bq2597x.tmh>

/* feature configuration */
unsigned int bat_ovp_th = 4550;        /* battery ovp threshold (mV) */
unsigned int bat_ovp_alarm_th = 4525;  /* battery ovp alarm threshold (mV) */
unsigned int bat_ocp_th = 8000;
unsigned int bat_ocp_alarm_th = 7500;
unsigned int bus_ovp_th = 12000;        /* IIN ovp threshold (mV) */
unsigned int bus_ovp_alarm_th = 11000;  /* IIN ovp alarm threshold (mV) */
unsigned int bus_ocp_th = 3750;        /* IIN ocp threshold (mA) */
unsigned int bus_ocp_alarm_th = 3500;  /* IIN ocp alarm threshold */
unsigned int bat_ucp_alarm_th = 2000;
unsigned char bat_therm_th = 0x15;
unsigned char bus_therm_th = 0x15;
unsigned int die_term_th = 145;
unsigned int ac_ovp_th = 14;
unsigned int sense_resistor_mohm = 2;

/* protection enable/disable */
//ti,bq2597x_charger,bat-ovp-disable
bool bat_ovp_disable = false;          /* disable battery voltage OVP */
//ti,bq2597x_charger,bat-ovp-alarm-disable
bool bat_ovp_alarm_disable = false;          /* disable battery voltage (float) regulation */
//ti,bq2597x_charger,bat-ocp-disable
bool bat_ocp_disable = false;           /* disable input current OCP */
//ti,bq2597x_charger,bat-ocp-alarm-disable
bool bat_ocp_alarm_disable = false;           /* disable input current regulation */
//ti,bq2597x_charger,bat-ucp-alarm-disable
bool bat_ucp_alarm_disable = false;          /* disable BUS temperature monitor (prot/alarm) */
//ti,bq2597x_charger,bat-ucp-disable
bool bat_ucp_disable = false;          /* disable BAT temperature monitor (prot/alarm) */

//ti,bq2597x_charger,bus-ovp-alarm-disable
bool bus_ovp_alarm_disable = false;         /* disable die temperature protection */
//ti,bq2597x_charger,bus-ocp-disable
bool bus_ocp_disable = false;          /* disable die temperature regulation */
//ti,bq2597x_charger,bus-ocp-alarm-disable
bool bus_ocp_alarm_disable = false;      /* disable reverse current protection */

//ti,bq2597x_charger,bat-therm-disable
bool bat_therm_disable = false;      /* disable reverse current protection */
//ti,bq2597x_charger,bus-therm-disable
bool bus_therm_disable = false;      /* disable reverse current protection */
//ti,bq2597x_charger,die-therm-disable
bool die_therm_disable = false;      /* disable reverse current protection */

enum {
	ADC_IBUS,
	ADC_VBUS,
	ADC_VAC,
	ADC_VOUT,
	ADC_VBAT,
	ADC_IBAT,
	ADC_TBUS,
	ADC_TBAT,
	ADC_TDIE,
	ADC_MAX_NUM,
};

#define BQ25970_ROLE_STDALONE 0
#define BQ25970_ROLE_SLAVE 1
#define BQ25970_ROLE_MASTER 2

enum {
    BQ25970_STDALONE,
    BQ25970_SLAVE,
    BQ25970_MASTER,
};

NTSTATUS bq2597x_mode_data[] = {
    [BQ25970_STDALONE] = BQ25970_STDALONE,
    [BQ25970_MASTER] = BQ25970_ROLE_MASTER,
    [BQ25970_SLAVE] = BQ25970_ROLE_SLAVE,
};

#define	BAT_OVP_ALARM		BIT(7)
#define BAT_OCP_ALARM		BIT(6)
#define	BUS_OVP_ALARM		BIT(5)
#define	BUS_OCP_ALARM		BIT(4)
#define	BAT_UCP_ALARM		BIT(3)
#define	VBUS_INSERT		BIT(2)
#define VBAT_INSERT		BIT(1)
#define	ADC_DONE		BIT(0)

#define BAT_OVP_FAULT		BIT(7)
#define BAT_OCP_FAULT		BIT(6)
#define BUS_OVP_FAULT		BIT(5)
#define BUS_OCP_FAULT		BIT(4)
#define TBUS_TBAT_ALARM		BIT(3)
#define TS_BAT_FAULT		BIT(2)
#define	TS_BUS_FAULT		BIT(1)
#define	TS_DIE_FAULT		BIT(0)

#define BIT(n) (1<<(n))

/*below used for comm with other module*/
#define	BAT_OVP_FAULT_SHIFT			0
#define	BAT_OCP_FAULT_SHIFT			1
#define	BUS_OVP_FAULT_SHIFT			2
#define	BUS_OCP_FAULT_SHIFT			3
#define	BAT_THERM_FAULT_SHIFT			4
#define	BUS_THERM_FAULT_SHIFT			5
#define	DIE_THERM_FAULT_SHIFT			6

#define	BAT_OVP_FAULT_MASK		(1 << BAT_OVP_FAULT_SHIFT)
#define	BAT_OCP_FAULT_MASK		(1 << BAT_OCP_FAULT_SHIFT)
#define	BUS_OVP_FAULT_MASK		(1 << BUS_OVP_FAULT_SHIFT)
#define	BUS_OCP_FAULT_MASK		(1 << BUS_OCP_FAULT_SHIFT)
#define	BAT_THERM_FAULT_MASK		(1 << BAT_THERM_FAULT_SHIFT)
#define	BUS_THERM_FAULT_MASK		(1 << BUS_THERM_FAULT_SHIFT)
#define	DIE_THERM_FAULT_MASK		(1 << DIE_THERM_FAULT_SHIFT)

#define	BAT_OVP_ALARM_SHIFT			0
#define	BAT_OCP_ALARM_SHIFT			1
#define	BUS_OVP_ALARM_SHIFT			2
#define	BUS_OCP_ALARM_SHIFT			3
#define	BAT_THERM_ALARM_SHIFT			4
#define	BUS_THERM_ALARM_SHIFT			5
#define	DIE_THERM_ALARM_SHIFT			6
#define BAT_UCP_ALARM_SHIFT			7

#define	BAT_OVP_ALARM_MASK		(1 << BAT_OVP_ALARM_SHIFT)
#define	BAT_OCP_ALARM_MASK		(1 << BAT_OCP_ALARM_SHIFT)
#define	BUS_OVP_ALARM_MASK		(1 << BUS_OVP_ALARM_SHIFT)
#define	BUS_OCP_ALARM_MASK		(1 << BUS_OCP_ALARM_SHIFT)
#define	BAT_THERM_ALARM_MASK		(1 << BAT_THERM_ALARM_SHIFT)
#define	BUS_THERM_ALARM_MASK		(1 << BUS_THERM_ALARM_SHIFT)
#define	DIE_THERM_ALARM_MASK		(1 << DIE_THERM_ALARM_SHIFT)
#define	BAT_UCP_ALARM_MASK		(1 << BAT_UCP_ALARM_SHIFT)

#define VBAT_REG_STATUS_SHIFT			0
#define IBAT_REG_STATUS_SHIFT			1

#define VBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)
#define IBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)

enum hvdcp3_type {
    HVDCP3_NONE = 0,
    HVDCP3_CLASSA_18W,
    HVDCP3_CLASSB_27W,
};

#define BUS_OVP_FOR_QC			10000
#define BUS_OVP_ALARM_FOR_QC			9500
#define BUS_OCP_FOR_QC_CLASS_A			3250
#define BUS_OCP_ALARM_FOR_QC_CLASS_A			2000
#define BUS_OCP_FOR_QC_CLASS_B			3750
#define BUS_OCP_ALARM_FOR_QC_CLASS_B			2800

struct bq2597x_cfg {
    bool bat_ovp_disable;
    bool bat_ocp_disable;
    bool bat_ovp_alm_disable;
    bool bat_ocp_alm_disable;

    int bat_ovp_th;
    int bat_ovp_alm_th;
    int bat_ocp_th;
    int bat_ocp_alm_th;

    bool bus_ovp_alm_disable;
    bool bus_ocp_disable;
    bool bus_ocp_alm_disable;

    int bus_ovp_th;
    int bus_ovp_alm_th;
    int bus_ocp_th;
    int bus_ocp_alm_th;

    bool bat_ucp_alm_disable;

    int bat_ucp_alm_th;
    int ac_ovp_th;

    bool bat_therm_disable;
    bool bus_therm_disable;
    bool die_therm_disable;

    int bat_therm_th; /*in %*/
    int bus_therm_th; /*in %*/
    int die_therm_th; /*in degC*/

    int sense_r_mohm;
};

static int bq_debug_flag;

#define ADC_REG_BASE 0x16

NTSTATUS
DriverEntry(
    IN PDRIVER_OBJECT  DriverObject,
    IN PUNICODE_STRING RegistryPath
)
{
    NTSTATUS               status = STATUS_SUCCESS;
    WDF_DRIVER_CONFIG      config;
    WDF_OBJECT_ATTRIBUTES  attributes;
    WPP_INIT_TRACING(DriverObject, RegistryPath);
    TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Driver Entry\n");

    WDF_DRIVER_CONFIG_INIT(&config, Bq2597xEvtDeviceAdd);

    WDF_OBJECT_ATTRIBUTES_INIT(&attributes);

    //
    // Create a framework driver object to represent our driver.
    //

    status = WdfDriverCreate(DriverObject,
        RegistryPath,
        &attributes,
        &config,
        WDF_NO_HANDLE
    );

    if (!NT_SUCCESS(status))
    {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "WdfDriverCreate failed with status 0x%x\n", status);
        WPP_CLEANUP(DriverObject);
    }

    return status;
}

NTSTATUS bq2597x_write_reg(PBQ2597X_CONTEXT pDevice, int reg, UINT8 data)
{
    INT32 status;
    UINT8 buf[2];

    buf[0] = reg;
    buf[1] = data;

    status = SpbWriteDataSynchronously(&pDevice->I2CContext, buf, sizeof(buf));

    return 0;
}

NTSTATUS bq2597x_read_reg(
    IN PBQ2597X_CONTEXT pDevice,
    UINT8 reg,
    UINT8* data
) {
    INT32 status;
    UINT8 raw_data = 0;

    status = SpbWriteRead(&pDevice->I2CContext, &reg, sizeof(reg), &raw_data, sizeof(raw_data), 0);
    *data = (UINT8)(raw_data);

    return 0;
}

NTSTATUS bq2597x_read_reg_word(
    IN PBQ2597X_CONTEXT pDevice,
    UINT8 reg,
    UINT16* data
) {
    INT32 status;

    status = SpbWriteRead(&pDevice->I2CContext, &reg, sizeof(reg), data, sizeof(data), 0);

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "reg=%d data=%d", reg, *data);

    return 0;
}

NTSTATUS bq2597x_update_reg(
    PBQ2597X_CONTEXT pDevice, 
    UINT8 reg,
    UINT8 mask, 
    UINT8 val)
{
    NTSTATUS status;
    UINT8 temp_val, data;

    status = bq2597x_read_reg(pDevice, reg, &data);

    if (!NT_SUCCESS(status)) {
        return status;
    }

    temp_val = data & ~mask;
    temp_val |= val & mask;

    if (data == temp_val)
    {
        status = STATUS_SUCCESS;
        return status;
    }

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "data=%d temp_val=%d mask=%d ~mask=%d val=%d", data, temp_val, mask, ~mask, val);

    status = bq2597x_write_reg(pDevice, reg, temp_val);

    return status;
}

void udelay(ULONG usec) {
    LARGE_INTEGER Interval;
    Interval.QuadPart = -10 * (LONGLONG)usec;
    KeDelayExecutionThread(KernelMode, false, &Interval);
}

void msleep(ULONG msec) {
    udelay(msec * 1000);
}

/**
 * bq2597x device driver control routines 
 */
NTSTATUS bq2597x_enable_charge(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;
    if (enable)
        val = BQ2597X_CHG_ENABLE;
    else val = BQ2597X_CHG_DISABLE;

    val <<= BQ2597X_CHG_EN_SHIFT;
    return bq2597x_update_reg(pDevice, BQ2597X_REG_0C, BQ2597X_CHG_EN_MASK, val);
}

NTSTATUS bq2597x_check_charge_enabled(PBQ2597X_CONTEXT pDevice, bool* enabled)
{
    NTSTATUS ret;
    UINT8 val;
    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_0C, &val);
    if (!ret)
        *enabled = !!(val & BQ2597X_CHG_EN_MASK);
    return ret;
}

NTSTATUS bq2597x_enable_wdt(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_WATCHDOG_ENABLE;
    else
        val = BQ2597X_WATCHDOG_DISABLE;

    val <<= BQ2597X_WATCHDOG_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_0B, BQ2597X_WATCHDOG_DIS_MASK, val);
}

NTSTATUS bq2597x_set_wdt(PBQ2597X_CONTEXT pDevice, int ms)
{
    UINT8 val;

    switch (ms) {
    case 500:
        val = BQ2597X_WATCHDOG_0P5S;
        break;
    case 1000:
        val = BQ2597X_WATCHDOG_1S;
        break;
    case 5000:
        val = BQ2597X_WATCHDOG_5S;
        break;
    case 30000:
        val = BQ2597X_WATCHDOG_30S;
        break;
    default:
        val = BQ2597X_WATCHDOG_30S;
        break;
    }

    val <<= BQ2597X_WATCHDOG_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_0B, BQ2597X_WATCHDOG_MASK, val);
}

NTSTATUS bq2597x_enable_batovp(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_BAT_OVP_ENABLE;
    else
        val = BQ2597X_BAT_OVP_DISABLE;

    val <<= BQ2597X_BAT_OVP_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_00, BQ2597X_BAT_OVP_DIS_MASK, val);
}

NTSTATUS bq2597x_set_batovp_th(PBQ2597X_CONTEXT pDevice, UINT8 threshold)
{
    UINT8 val;

    if (threshold < BQ2597X_BAT_OVP_BASE)
        threshold = BQ2597X_BAT_OVP_BASE;

    val = (threshold - BQ2597X_BAT_OVP_BASE) / BQ2597X_BAT_OVP_LSB;

    val <<= BQ2597X_BAT_OVP_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_00, BQ2597X_BAT_OVP_MASK, val);
}

NTSTATUS bq2597x_enable_batovp_alarm(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_BAT_OVP_ALM_ENABLE;
    else
        val = BQ2597X_BAT_OVP_ALM_DISABLE;

    val <<= BQ2597X_BAT_OVP_ALM_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_01, BQ2597X_BAT_OVP_ALM_DIS_MASK, val);
}

NTSTATUS bq2597x_set_batovp_alarm_th(PBQ2597X_CONTEXT pDevice, UINT8 threshold)
{
    UINT8 val;

    if (threshold < BQ2597X_BAT_OVP_ALM_BASE)
        threshold = BQ2597X_BAT_OVP_ALM_BASE;

    val = (threshold - BQ2597X_BAT_OVP_ALM_BASE) / BQ2597X_BAT_OVP_ALM_LSB;

    val <<= BQ2597X_BAT_OVP_ALM_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_01, BQ2597X_BAT_OVP_ALM_MASK, val);
}

NTSTATUS bq2597x_enable_batocp(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_BAT_OCP_ENABLE;
    else
        val = BQ2597X_BAT_OCP_DISABLE;

    val <<= BQ2597X_BAT_OCP_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_02, BQ2597X_BAT_OCP_DIS_MASK, val);
}

NTSTATUS bq2597x_set_batocp_th(PBQ2597X_CONTEXT pDevice, UINT8 threshold)
{
    UINT8 val;

    if (threshold < BQ2597X_BAT_OCP_BASE)
        threshold = BQ2597X_BAT_OCP_BASE;

    val = (threshold - BQ2597X_BAT_OCP_BASE) / BQ2597X_BAT_OCP_LSB;

    val <<= BQ2597X_BAT_OCP_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_02, BQ2597X_BAT_OCP_MASK, val);
}

NTSTATUS bq2597x_enable_batocp_alarm(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_BAT_OCP_ALM_ENABLE;
    else
        val = BQ2597X_BAT_OCP_ALM_DISABLE;

    val <<= BQ2597X_BAT_OCP_ALM_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_03, BQ2597X_BAT_OCP_ALM_DIS_MASK, val);
}

NTSTATUS bq2597x_set_batocp_alarm_th(PBQ2597X_CONTEXT pDevice, int threshold)
{
    UINT8 val;

    if (threshold < BQ2597X_BAT_OCP_ALM_BASE)
        threshold = BQ2597X_BAT_OCP_ALM_BASE;

    val = (threshold - BQ2597X_BAT_OCP_ALM_BASE) / BQ2597X_BAT_OCP_ALM_LSB;

    val <<= BQ2597X_BAT_OCP_ALM_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_03, BQ2597X_BAT_OCP_ALM_MASK, val);
}

NTSTATUS bq2597x_set_busovp_th(PBQ2597X_CONTEXT pDevice, int threshold)
{
    UINT8 val;

    if (threshold < BQ2597X_BUS_OVP_BASE)
        threshold = BQ2597X_BUS_OVP_BASE;

    val = (threshold - BQ2597X_BUS_OVP_BASE) / BQ2597X_BUS_OVP_LSB;

    val <<= BQ2597X_BUS_OVP_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_06, BQ2597X_BUS_OVP_MASK, val);
}

NTSTATUS bq2597x_enable_busovp_alarm(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_BUS_OVP_ALM_ENABLE;
    else
        val = BQ2597X_BUS_OVP_ALM_DISABLE;

    val <<= BQ2597X_BUS_OVP_ALM_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_07, BQ2597X_BUS_OVP_ALM_DIS_MASK, val);
}

NTSTATUS bq2597x_set_busovp_alarm_th(PBQ2597X_CONTEXT pDevice, int threshold)
{
    UINT8 val;

    if (threshold < BQ2597X_BUS_OVP_ALM_BASE)
        threshold = BQ2597X_BUS_OVP_ALM_BASE;

    val = (threshold - BQ2597X_BUS_OVP_ALM_BASE) / BQ2597X_BUS_OVP_ALM_LSB;

    val <<= BQ2597X_BUS_OVP_ALM_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_07, BQ2597X_BUS_OVP_ALM_MASK, val);
}

NTSTATUS bq2597x_enable_busocp(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_BUS_OCP_ENABLE;
    else
        val = BQ2597X_BUS_OCP_DISABLE;

    val <<= BQ2597X_BUS_OCP_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_08, BQ2597X_BUS_OCP_DIS_MASK, val);
}

NTSTATUS bq2597x_set_busocp_th(PBQ2597X_CONTEXT pDevice, int threshold)
{
    UINT8 val;

    if (threshold < BQ2597X_BUS_OCP_BASE)
        threshold = BQ2597X_BUS_OCP_BASE;

    val = (threshold - BQ2597X_BUS_OCP_BASE) / BQ2597X_BUS_OCP_LSB;

    val <<= BQ2597X_BUS_OCP_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_08, BQ2597X_BUS_OCP_MASK, val);
}

NTSTATUS bq2597x_enable_busocp_alarm(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_BUS_OCP_ALM_ENABLE;
    else
        val = BQ2597X_BUS_OCP_ALM_DISABLE;

    val <<= BQ2597X_BUS_OCP_ALM_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_09, BQ2597X_BUS_OCP_ALM_DIS_MASK, val);
}

NTSTATUS bq2597x_set_busocp_alarm_th(PBQ2597X_CONTEXT pDevice, int threshold)
{
    UINT8 val;

    if (threshold < BQ2597X_BUS_OCP_ALM_BASE)
        threshold = BQ2597X_BUS_OCP_ALM_BASE;

    val = (threshold - BQ2597X_BUS_OCP_ALM_BASE) / BQ2597X_BUS_OCP_ALM_LSB;

    val <<= BQ2597X_BUS_OCP_ALM_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_09, BQ2597X_BUS_OCP_ALM_MASK, val);
}

NTSTATUS bq2597x_enable_batucp_alarm(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_BAT_UCP_ALM_ENABLE;
    else
        val = BQ2597X_BAT_UCP_ALM_DISABLE;

    val <<= BQ2597X_BAT_UCP_ALM_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_04, BQ2597X_BAT_UCP_ALM_DIS_MASK, val);
}

NTSTATUS bq2597x_set_batucp_alarm_th(PBQ2597X_CONTEXT pDevice, int threshold)
{
    UINT8 val;

    if (threshold < BQ2597X_BAT_UCP_ALM_BASE)
        threshold = BQ2597X_BAT_UCP_ALM_BASE;

    val = (threshold - BQ2597X_BAT_UCP_ALM_BASE) / BQ2597X_BAT_UCP_ALM_LSB;

    val <<= BQ2597X_BAT_UCP_ALM_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_04, BQ2597X_BAT_UCP_ALM_MASK, val);
}

NTSTATUS bq2597x_set_acovp_th(PBQ2597X_CONTEXT pDevice, int threshold)
{
    UINT8 val;

    if (threshold < BQ2597X_AC_OVP_BASE)
        threshold = BQ2597X_AC_OVP_BASE;

    if (threshold == BQ2597X_AC_OVP_6P5V)
        val = 0x07;
    else
        val = (threshold - BQ2597X_AC_OVP_BASE) / BQ2597X_AC_OVP_LSB;

    val <<= BQ2597X_AC_OVP_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_05, BQ2597X_AC_OVP_MASK, val);
}

NTSTATUS bq2597x_set_vdrop_th(PBQ2597X_CONTEXT pDevice, int threshold)
{
    UINT8 val;

    if (threshold == 300)
        val = BQ2597X_VDROP_THRESHOLD_300MV;
    else
        val = BQ2597X_VDROP_THRESHOLD_400MV;

    val <<= BQ2597X_VDROP_THRESHOLD_SET_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_05, BQ2597X_VDROP_THRESHOLD_SET_MASK, val);
}

NTSTATUS bq2597x_set_vdrop_deglitch(PBQ2597X_CONTEXT pDevice, int us)
{
    UINT8 val;

    if (us == 8)
        val = BQ2597X_VDROP_DEGLITCH_8US;
    else
        val = BQ2597X_VDROP_DEGLITCH_5MS;

    val <<= BQ2597X_VDROP_DEGLITCH_SET_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_05, BQ2597X_VDROP_DEGLITCH_SET_MASK, val);
}

NTSTATUS bq2597x_enable_bat_therm(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_TSBAT_ENABLE;
    else
        val = BQ2597X_TSBAT_DISABLE;

    val <<= BQ2597X_TSBAT_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_0C, BQ2597X_TSBAT_DIS_MASK, val);
}

NTSTATUS bq2597x_set_bat_therm_th(PBQ2597X_CONTEXT pDevice, UINT8 threshold)
{
    return bq2597x_write_reg(pDevice, BQ2597X_REG_29, threshold);
}

NTSTATUS bq2597x_enable_bus_therm(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_TSBUS_ENABLE;
    else
        val = BQ2597X_TSBUS_DISABLE;

    val <<= BQ2597X_TSBUS_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_0C, BQ2597X_TSBUS_DIS_MASK, val);
}

NTSTATUS bq2597x_set_bus_therm_th(PBQ2597X_CONTEXT pDevice, UINT8 threshold)
{
    return bq2597x_write_reg(pDevice, BQ2597X_REG_28, threshold);
}

NTSTATUS bq2597x_enable_die_therm(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_TDIE_ENABLE;
    else
        val = BQ2597X_TDIE_DISABLE;

    val <<= BQ2597X_TDIE_DIS_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_0C, BQ2597X_TDIE_DIS_MASK, val);
}

NTSTATUS bq2597x_set_die_therm_th(PBQ2597X_CONTEXT pDevice, UINT8 threshold)
{
    UINT8 val;

    val = (threshold - BQ2597X_TDIE_ALM_BASE) * BQ2597X_TDIE_ALM_LSB;
    val <<= BQ2597X_TDIE_ALM_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_2A, BQ2597X_TDIE_ALM_MASK, val);
}

NTSTATUS bq2597x_enable_adc(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_ADC_ENABLE;
    else
        val = BQ2597X_ADC_DISABLE;

    val <<= BQ2597X_ADC_EN_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_14, BQ2597X_ADC_EN_MASK, val);
}

NTSTATUS bq2597x_set_adc_average(PBQ2597X_CONTEXT pDevice, bool avg)
{
    UINT8 val;

    if (avg)
        val = BQ2597X_ADC_AVG_ENABLE;
    else
        val = BQ2597X_ADC_AVG_DISABLE;

    val <<= BQ2597X_ADC_AVG_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_14, BQ2597X_ADC_AVG_MASK, val);
}

NTSTATUS bq2597x_set_adc_scanrate(PBQ2597X_CONTEXT pDevice, bool oneshot)
{
    UINT8 val;

    if (oneshot)
        val = BQ2597X_ADC_RATE_ONESHOT;
    else
        val = BQ2597X_ADC_RATE_CONTINOUS;

    val <<= BQ2597X_ADC_RATE_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_14, BQ2597X_ADC_EN_MASK, val);
}

NTSTATUS bq2597x_set_adc_bits(PBQ2597X_CONTEXT pDevice, int bits)
{
    UINT8 val;

    if (bits > 15)
        bits = 15;
    if (bits < 12)
        bits = 12;
    val = 15 - bits;

    val <<= BQ2597X_ADC_SAMPLE_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_14, BQ2597X_ADC_SAMPLE_MASK, val);
}

NTSTATUS bq2597x_get_adc_data(PBQ2597X_CONTEXT pDevice, int channel, int *result)
{
    NTSTATUS ret;
    UINT16 val;
    INT16 t;

    if (channel > ADC_MAX_NUM)
        return STATUS_INVALID_PARAMETER;

    ret = bq2597x_read_reg_word(pDevice, ADC_REG_BASE + (channel << 1), &val);
    if (ret < 0)
        return ret;
    t = val & 0xFF;
    t <<= 8;
    t |= (val >> 8) & 0xFF;
    *result = t;

    return 0;
}

NTSTATUS bq2597x_set_adc_scan(PBQ2597X_CONTEXT pDevice, int channel, bool enable)
{
    UINT8 reg;
    UINT8 mask;
    UINT8 shift;
    UINT8 val;

    if (channel > ADC_MAX_NUM)
        return STATUS_INVALID_PARAMETER;

    if (channel == ADC_IBUS) {
        reg = BQ2597X_REG_14;
        shift = BQ2597X_IBUS_ADC_DIS_SHIFT;
        mask = BQ2597X_IBUS_ADC_DIS_MASK;
    }
    else {
        reg = BQ2597X_REG_15;
        shift = 8 - channel;
        mask = 1 << shift;
    }

    if (enable)
        val = 0 << shift;
    else
        val = 1 << shift;

    return bq2597x_update_reg(pDevice, reg, mask, val);
}

NTSTATUS bq2597x_set_alarm_int_mask(PBQ2597X_CONTEXT pDevice, UINT8 mask)
{
    NTSTATUS ret;
    UINT8 val;

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_0F, &val);
    if (ret)
        return ret;

    val |= mask;

    ret = bq2597x_write_reg(pDevice, BQ2597X_REG_0F, val);

    return ret;
}

NTSTATUS bq2597x_clear_alarm_int_mask(PBQ2597X_CONTEXT pDevice, UINT8 mask)
{
    NTSTATUS ret;
    UINT8 val;

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_0F, &val);
    if (ret)
        return ret;

    val &= ~mask;

    ret = bq2597x_write_reg(pDevice, BQ2597X_REG_0F, val);

    return ret;
}

NTSTATUS bq2597x_set_fault_int_mask(PBQ2597X_CONTEXT pDevice, UINT8 mask)
{
    NTSTATUS ret;
    UINT8 val;

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_12, &val);
    if (ret)
        return ret;

    val |= mask;

    ret = bq2597x_write_reg(pDevice, BQ2597X_REG_12, val);

    return ret;
}

NTSTATUS bq2597x_clear_fault_int_mask(PBQ2597X_CONTEXT pDevice, UINT8 mask)
{
    NTSTATUS ret;
    UINT8 val;

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_12, &val);
    if (ret)
        return ret;

    val &= ~mask;

    ret = bq2597x_write_reg(pDevice, BQ2597X_REG_12, val);

    return ret;
}

NTSTATUS bq2597x_set_sense_resistor(PBQ2597X_CONTEXT pDevice, int r_mohm)
{
    UINT8 val;

    if (r_mohm == 2)
        val = BQ2597X_SET_IBAT_SNS_RES_2MHM;
    else if (r_mohm == 5)
        val = BQ2597X_SET_IBAT_SNS_RES_5MHM;
    else
        return STATUS_INVALID_PARAMETER;

    val <<= BQ2597X_SET_IBAT_SNS_RES_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_2B, BQ2597X_SET_IBAT_SNS_RES_MASK, val);
}

NTSTATUS bq2597x_set_ibus_ucp_thr(PBQ2597X_CONTEXT pDevice, int ibus_ucp_thr)
{
    UINT8 val;

    if (ibus_ucp_thr == 300)
        val = BQ2597X_IBUS_UCP_RISE_300MA;
    else if (ibus_ucp_thr == 500)
        val = BQ2597X_IBUS_UCP_RISE_500MA;
    else
        return STATUS_INVALID_PARAMETER;

    val <<= BQ2597X_IBUS_UCP_RISE_TH_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_2B, BQ2597X_IBUS_UCP_RISE_TH_MASK, val);
}

NTSTATUS bq2597x_enable_regulation(PBQ2597X_CONTEXT pDevice, bool enable)
{
    UINT8 val;

    if (enable)
        val = BQ2597X_EN_REGULATION_ENABLE;
    else
        val = BQ2597X_EN_REGULATION_DISABLE;

    val <<= BQ2597X_EN_REGULATION_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_2B, BQ2597X_EN_REGULATION_MASK, val);
}

NTSTATUS bq2597x_set_ss_timeout(PBQ2597X_CONTEXT pDevice, int timeout)
{
    UINT8 val;

    switch (timeout) {
    case 0:
        val = BQ2597X_SS_TIMEOUT_DISABLE;
        break;
    case 12:
        val = BQ2597X_SS_TIMEOUT_12P5MS;
        break;
    case 25:
        val = BQ2597X_SS_TIMEOUT_25MS;
        break;
    case 50:
        val = BQ2597X_SS_TIMEOUT_50MS;
        break;
    case 100:
        val = BQ2597X_SS_TIMEOUT_100MS;
        break;
    case 400:
        val = BQ2597X_SS_TIMEOUT_400MS;
        break;
    case 1500:
        val = BQ2597X_SS_TIMEOUT_1500MS;
        break;
    case 100000:
        val = BQ2597X_SS_TIMEOUT_100000MS;
        break;
    default:
        val = BQ2597X_SS_TIMEOUT_DISABLE;
        break;
    }

    val <<= BQ2597X_SS_TIMEOUT_SET_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_2B, BQ2597X_SS_TIMEOUT_SET_MASK, val);
}

NTSTATUS bq2597x_set_ibat_reg_th(PBQ2597X_CONTEXT pDevice, int th_ma)
{
    UINT8 val;

    if (th_ma == 200)
        val = BQ2597X_IBAT_REG_200MA;
    else if (th_ma == 300)
        val = BQ2597X_IBAT_REG_300MA;
    else if (th_ma == 400)
        val = BQ2597X_IBAT_REG_400MA;
    else if (th_ma == 500)
        val = BQ2597X_IBAT_REG_500MA;
    else
        val = BQ2597X_IBAT_REG_500MA;

    val <<= BQ2597X_IBAT_REG_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_2C, BQ2597X_IBAT_REG_MASK, val);
}

NTSTATUS bq2597x_set_vbat_reg_th(PBQ2597X_CONTEXT pDevice, int th_mv)
{
    UINT8 val;

    switch (th_mv) {
    case 50:
        val = BQ2597X_VBAT_REG_50MV;
        break;
    case 100:
        val = BQ2597X_VBAT_REG_100MV;
        break;
    case 150:
        val = BQ2597X_VBAT_REG_150MV;
        break;
    default:
        val = BQ2597X_VBAT_REG_200MV;
        break;
    }
    val <<= BQ2597X_VBAT_REG_SHIFT;

    return bq2597x_update_reg(pDevice, BQ2597X_REG_2C, BQ2597X_VBAT_REG_MASK, val);
}

NTSTATUS bq2597x_check_reg_status(PBQ2597X_CONTEXT pDevice)
{
    NTSTATUS ret;
    UINT8 val;

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_2C, &val);
    if (!ret) {
        (pDevice->vbat_reg) = !!(val & BQ2597X_VBAT_REG_ACTIVE_STAT_MASK);
        (pDevice->ibat_reg) = !!(val & BQ2597X_IBAT_REG_ACTIVE_STAT_MASK);
    }

    return ret;
}

NTSTATUS bq2597x_get_work_mode(PBQ2597X_CONTEXT pDevice, int* mode)
{
    NTSTATUS ret;
    UINT8 val;

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_0C, &val);

    if (ret) {
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "Failed to read operation mode register\n");
        return ret;
    }

    val = (val & BQ2597X_MS_MASK) >> BQ2597X_MS_SHIFT;
    if (val == BQ2597X_MS_MASTER)
        *mode = BQ25970_ROLE_MASTER;
    else if (val == BQ2597X_MS_SLAVE)
        *mode = BQ25970_ROLE_SLAVE;
    else
        *mode = BQ25970_ROLE_STDALONE;

    return ret;
}

NTSTATUS bq2597x_detect_device(PBQ2597X_CONTEXT pDevice)
{
    NTSTATUS ret;
    UINT8 data;

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_13, &data);
    if (ret == 0) {
        (pDevice->part_no) = (data & BQ2597X_DEV_ID_MASK);
        (pDevice->part_no) >>= BQ2597X_DEV_ID_SHIFT;
    }

    return ret;
}

#define RUNNING_PERIOD_S	(60 * 1000)

/*static void bq2597x_monitor_work(struct work_struct* work)
{
    struct bq2597x* bq = container_of(work, struct bq2597x,
        monitor_work.work);
    bq2597x_dump_reg(bq);
    schedule_delayed_work(&bq->monitor_work,
        msecs_to_jiffies(RUNNING_PERIOD_S));
}*/ // TODO: FIND OUT HOW TO CONVERT THIS

NTSTATUS bq2597x_init_protection(PBQ2597X_CONTEXT pDevice)
{
    NTSTATUS ret;
    
    ret = bq2597x_enable_batovp(pDevice, !bat_ovp_disable);
    ret = bq2597x_enable_batocp(pDevice, !bat_ocp_disable);
    ret = bq2597x_enable_batovp_alarm(pDevice, !bat_ovp_alarm_disable);
    ret = bq2597x_enable_batocp_alarm(pDevice, !bat_ocp_alarm_disable);
    ret = bq2597x_enable_batucp_alarm(pDevice, !bat_ucp_alarm_disable);
    ret = bq2597x_enable_busovp_alarm(pDevice, !bus_ovp_alarm_disable);
    ret = bq2597x_enable_busocp(pDevice, !bus_ocp_disable);
    ret = bq2597x_enable_busocp_alarm(pDevice, !bus_ocp_alarm_disable);
    ret = bq2597x_enable_bat_therm(pDevice, !bat_therm_disable);
    ret = bq2597x_enable_bus_therm(pDevice, !bus_therm_disable);
    ret = bq2597x_enable_die_therm(pDevice, !die_therm_disable);
    ret = bq2597x_set_batovp_th(pDevice, !bat_ovp_th);
    ret = bq2597x_set_batovp_alarm_th(pDevice, !bat_ovp_alarm_th);
    ret = bq2597x_set_batocp_th(pDevice, !bat_ocp_th);
    ret = bq2597x_set_batocp_alarm_th(pDevice, !bat_ocp_alarm_th);
    ret = bq2597x_set_busovp_th(pDevice, !bus_ovp_th);
    ret = bq2597x_set_busovp_alarm_th(pDevice, !bus_ovp_alarm_th);
    ret = bq2597x_set_busocp_th(pDevice, !bus_ocp_th);
    ret = bq2597x_set_busocp_alarm_th(pDevice, !bus_ocp_alarm_th);
    ret = bq2597x_set_batucp_alarm_th(pDevice, !bat_ucp_alarm_th);
    ret = bq2597x_set_bat_therm_th(pDevice, !bat_therm_th);
    ret = bq2597x_set_bus_therm_th(pDevice, !bus_therm_th);
    ret = bq2597x_set_die_therm_th(pDevice, !die_term_th);
    ret = bq2597x_set_acovp_th(pDevice, !ac_ovp_th);

    return 0;
}

NTSTATUS bq2597x_set_bus_protection(PBQ2597X_CONTEXT pDevice, int hvdcp3_type)
{

    int data = BUS_OVP_FOR_QC;
    int data2 = BUS_OVP_ALARM_FOR_QC;
    int data3 = 0;
    int data4 = 0;
    switch (hvdcp3_type) {
    case HVDCP3_CLASSA_18W:
        data3 = BUS_OCP_FOR_QC_CLASS_A;
        data4 = BUS_OCP_ALARM_FOR_QC_CLASS_A;
        break;
    case HVDCP3_CLASSB_27W:
        data3 = BUS_OCP_FOR_QC_CLASS_B;
        data4 = BUS_OCP_ALARM_FOR_QC_CLASS_B;
        break;
    default:
        data = bus_ovp_th;
        data2 = bus_ovp_alarm_th;
        data3 = bus_ocp_th;
        data4 = bus_ocp_alarm_th;
        break;
    }

    bq2597x_set_busovp_th(pDevice, data);
    bq2597x_set_busovp_alarm_th(pDevice, data2);
    bq2597x_set_busocp_th(pDevice, data3);
    bq2597x_set_busocp_alarm_th(pDevice, data4);

    return 0;
}

NTSTATUS bq2597x_init_adc(PBQ2597X_CONTEXT pDevice)
{
    bq2597x_set_adc_scanrate(pDevice, false);
    bq2597x_set_adc_bits(pDevice, 13);
    bq2597x_set_adc_average(pDevice, true);
    bq2597x_set_adc_scan(pDevice, ADC_IBUS, true);
    bq2597x_set_adc_scan(pDevice, ADC_VBUS, true);
    bq2597x_set_adc_scan(pDevice, ADC_VOUT, false);
    bq2597x_set_adc_scan(pDevice, ADC_VBAT, true);
    bq2597x_set_adc_scan(pDevice, ADC_IBAT, false);
    bq2597x_set_adc_scan(pDevice, ADC_TBUS, false);
    bq2597x_set_adc_scan(pDevice, ADC_TBAT, false);
    bq2597x_set_adc_scan(pDevice, ADC_TDIE, false);
    bq2597x_set_adc_scan(pDevice, ADC_VAC, true);

    bq2597x_enable_adc(pDevice, true);

    return 0;
}

NTSTATUS bq2597x_init_int_src(PBQ2597X_CONTEXT pDevice)
{
    NTSTATUS ret;
    /*TODO:be careful ts bus and ts bat alarm bit mask is in
     *	fault mask register, so you need call
     *	bq2597x_set_fault_int_mask for tsbus and tsbat alarm
     */
    ret = bq2597x_set_alarm_int_mask(pDevice, ADC_DONE
        | BAT_OCP_ALARM | BAT_UCP_ALARM
        | BAT_OVP_ALARM);
    if (ret) {
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "failed to set alarm mask:%d\n", ret);
        return ret;
    }
    //#if 0
    ret = bq2597x_set_fault_int_mask(pDevice,
        TS_BUS_FAULT | TS_DIE_FAULT | TS_BAT_FAULT | BAT_OCP_FAULT);
    if (ret) {
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "failed to set fault mask:%d\n", ret);
        return ret;
    }
    //#endif
    return ret;
}

NTSTATUS bq2597x_init_regulation(PBQ2597X_CONTEXT pDevice)
{
    bq2597x_set_ibat_reg_th(pDevice, 200);
    bq2597x_set_vbat_reg_th(pDevice, 50);

    bq2597x_set_vdrop_deglitch(pDevice, 5000);
    bq2597x_set_vdrop_th(pDevice, 400);

    bq2597x_enable_regulation(pDevice, true);

    return 0;
}

NTSTATUS bq2597x_init_device(PBQ2597X_CONTEXT pDevice)
{
    bq2597x_enable_wdt(pDevice, false);

    bq2597x_set_ss_timeout(pDevice, 100000);
    bq2597x_set_ibus_ucp_thr(pDevice, 300);
    bq2597x_set_sense_resistor(pDevice, sense_resistor_mohm);

    bq2597x_init_protection(pDevice);
    bq2597x_init_adc(pDevice);
    bq2597x_init_int_src(pDevice);

    bq2597x_init_regulation(pDevice);

    return 0;
}

NTSTATUS bq2597x_set_present(PBQ2597X_CONTEXT pDevice, bool present)
{
    (pDevice->usb_present) = present;

    if (present)
        bq2597x_init_device(pDevice);
    return 0;
}

/*static ssize_t bq2597x_show_registers(struct device* dev,
    struct device_attribute* attr, char* buf)
{
    PBQ2597X_CONTEXT pDevice = dev_get_drvdata(dev);
    u8 addr;
    u8 val;
    u8 tmpbuf[300];
    int len;
    int idx = 0;
    NTSTATUS ret;

    idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq25970");
    for (addr = 0x0; addr <= 0x2A; addr++) {
        ret = bq2597x_read_byte(bq, addr, &val);
        if (ret == 0) {
            len = snprintf(tmpbuf, PAGE_SIZE - idx,
                "Reg[%.2X] = 0x%.2x\n", addr, val);
            memcpy(&buf[idx], tmpbuf, len);
            idx += len;
        }
    }

    return idx;
}

static ssize_t bq2597x_store_register(struct device* dev,
    struct device_attribute* attr, const char* buf, size_t count)
{
    struct bq2597x* bq = dev_get_drvdata(dev);
    NTSTATUS ret;
    unsigned int reg;
    unsigned int val;

    ret = sscanf(buf, "%x %x", &reg, &val);
    if (ret == 2 && reg <= 0x2A)
        bq2597x_write_byte(bq, (unsigned char)reg, (unsigned char)val);

    return count;
}


static DEVICE_ATTR(registers, 0660, bq2597x_show_registers, bq2597x_store_register);*/

/* static void bq2597x_check_alarm_status(BQ2597X_CONTEXT pDevice) {}
static void bq2597x_check_fault_status(BQ2597X_CONTEXT pDevice) {}

NTSTATUS bq2597x_charger_get_property(struct power_supply* psy,
    enum power_supply_property psp,
    union power_supply_propval* val)
{
    PBQ2597X_CONTEXT pDevice = power_supply_get_drvdata(psy);
    int result;
    NTSTATUS ret;
    u8 reg_val;

    switch (psp) {
    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        bq2597x_check_charge_enabled(bq, &bq->charge_enabled);
        val->intval = bq->charge_enabled;
        break;
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = 0;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = bq->usb_present;
        break;
    case POWER_SUPPLY_PROP_TI_BATTERY_PRESENT:
        ret = bq2597x_read_byte(bq, BQ2597X_REG_0D, &reg_val);
        if (!ret)
            bq->batt_present = !!(reg_val & VBAT_INSERT);
        val->intval = bq->batt_present;
        break;
    case POWER_SUPPLY_PROP_TI_VBUS_PRESENT:
        ret = bq2597x_read_byte(bq, BQ2597X_REG_0D, &reg_val);
        if (!ret)
            bq->vbus_present = !!(reg_val & VBUS_INSERT);
        val->intval = bq->vbus_present;
        break;
    case POWER_SUPPLY_PROP_TI_BATTERY_VOLTAGE:
        ret = bq2597x_get_adc_data(bq, ADC_VBAT, &result);
        if (!ret)
            bq->vbat_volt = result;

        val->intval = bq->vbat_volt;
        break;
    case POWER_SUPPLY_PROP_TI_BATTERY_CURRENT:
        ret = bq2597x_get_adc_data(bq, ADC_IBAT, &result);
        if (!ret)
            bq->ibat_curr = result;

        val->intval = bq->ibat_curr;
        break;
    case POWER_SUPPLY_PROP_TI_BATTERY_TEMPERATURE:
        ret = bq2597x_get_adc_data(bq, ADC_TBAT, &result);
        if (!ret)
            bq->bat_temp = result;

        val->intval = bq->bat_temp;
        break;
    case POWER_SUPPLY_PROP_TI_BUS_VOLTAGE:
        ret = bq2597x_get_adc_data(bq, ADC_VBUS, &result);
        if (!ret)
            bq->vbus_volt = result;

        val->intval = bq->vbus_volt;
        break;
    case POWER_SUPPLY_PROP_TI_BUS_CURRENT:
        ret = bq2597x_get_adc_data(bq, ADC_IBUS, &result);
        if (!ret)
            bq->ibus_curr = result;

        val->intval = bq->ibus_curr;
        break;
    case POWER_SUPPLY_PROP_TI_BUS_TEMPERATURE:
        ret = bq2597x_get_adc_data(bq, ADC_TBUS, &result);
        if (!ret)
            bq->bus_temp = result;

        val->intval = bq->bus_temp;
        break;
    case POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE:
        ret = bq2597x_get_adc_data(bq, ADC_TDIE, &result);
        if (!ret)
            bq->die_temp = result;

        val->intval = bq->die_temp;
        break;
    case POWER_SUPPLY_PROP_TI_ALARM_STATUS:

        bq2597x_check_alarm_status(bq);

        val->intval = ((bq->bat_ovp_alarm << BAT_OVP_ALARM_SHIFT)
            | (bq->bat_ocp_alarm << BAT_OCP_ALARM_SHIFT)
            | (bq->bat_ucp_alarm << BAT_UCP_ALARM_SHIFT)
            | (bq->bus_ovp_alarm << BUS_OVP_ALARM_SHIFT)
            | (bq->bus_ocp_alarm << BUS_OCP_ALARM_SHIFT)
            | (bq->bat_therm_alarm << BAT_THERM_ALARM_SHIFT)
            | (bq->bus_therm_alarm << BUS_THERM_ALARM_SHIFT)
            | (bq->die_therm_alarm << DIE_THERM_ALARM_SHIFT));
        break;

    case POWER_SUPPLY_PROP_TI_FAULT_STATUS:
        bq2597x_check_fault_status(bq);

        val->intval = ((bq->bat_ovp_fault << BAT_OVP_FAULT_SHIFT)
            | (bq->bat_ocp_fault << BAT_OCP_FAULT_SHIFT)
            | (bq->bus_ovp_fault << BUS_OVP_FAULT_SHIFT)
            | (bq->bus_ocp_fault << BUS_OCP_FAULT_SHIFT)
            | (bq->bat_therm_fault << BAT_THERM_FAULT_SHIFT)
            | (bq->bus_therm_fault << BUS_THERM_FAULT_SHIFT)
            | (bq->die_therm_fault << DIE_THERM_FAULT_SHIFT));
        break;

    case POWER_SUPPLY_PROP_TI_REG_STATUS:
        bq2597x_check_reg_status(bq);
        val->intval = (bq->vbat_reg << VBAT_REG_STATUS_SHIFT) |
            (bq->ibat_reg << IBAT_REG_STATUS_SHIFT);
        break;
    case POWER_SUPPLY_PROP_TI_SET_BUS_PROTECTION_FOR_QC3:
        val->intval = 0;
        break;
    case POWER_SUPPLY_PROP_MODEL_NAME:
        ret = bq2597x_get_work_mode(bq, &bq->mode);
        if (ret) {
            val->strval = "unknown";
        }
        else {
            if (bq->mode == BQ25970_ROLE_MASTER)
                val->strval = "bq2597x-master";
            else if (bq->mode == BQ25970_ROLE_SLAVE)
                val->strval = "bq2597x-slave";
            else
                val->strval = "bq2597x-standalone";
        }
        break;
    default:
        return -EINVAL;

    }

    return 0;
}

NTSTATUS bq2597x_charger_set_property(struct power_supply* psy,
    enum power_supply_property prop,
    const union power_supply_propval* val)
{
    struct bq2597x* bq = power_supply_get_drvdata(psy);

    switch (prop) {
    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        bq2597x_enable_charge(bq, val->intval);
        bq2597x_check_charge_enabled(bq, &bq->charge_enabled);
        bq_info("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n",
            val->intval ? "enable" : "disable");
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        bq2597x_set_present(bq, !!val->intval);
        break;
    case POWER_SUPPLY_PROP_TI_SET_BUS_PROTECTION_FOR_QC3:
        bq2597x_set_bus_protection(bq, val->intval);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

NTSTATUS bq2597x_charger_is_writeable(struct power_supply* psy,
    enum power_supply_property prop)
{
    NTSTATUS ret;

    switch (prop) {
    case POWER_SUPPLY_PROP_CHARGING_ENABLED:
    case POWER_SUPPLY_PROP_TI_SET_BUS_PROTECTION_FOR_QC3:
        ret = 1;
        break;
    default:
        ret = 0;
        break;
    }
    return ret;
} */

static void bq2597x_charger_info(PBQ2597X_CONTEXT pDevice)
{
    int vbat = 0, vbus = 0, ibus = 0, vac = 0;

    bq2597x_get_adc_data(pDevice, ADC_VBAT, &vbat);
    bq2597x_get_adc_data(pDevice, ADC_VBUS, &vbus);
    bq2597x_get_adc_data(pDevice, ADC_IBUS, &ibus);
    bq2597x_get_adc_data(pDevice, ADC_VAC, &vac);
    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "charger info: vbat(%d), vbus(%d), ibus(%d), vac(%d)\n",
        vbat, vbus, ibus, vac);
}

static void bq2597x_check_alarm_status(PBQ2597X_CONTEXT pDevice)
{
    int ret;
    UINT8 flag = 0;
    UINT8 stat = 0;

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_08, &flag);
    if (!ret && (flag & BQ2597X_IBUS_UCP_FALL_FLAG_MASK))
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "UCP_FLAG =0x%02X\n",
            !!(flag & BQ2597X_IBUS_UCP_FALL_FLAG_MASK));

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_2D, &flag);
    if (!ret && (flag & BQ2597X_VDROP_OVP_FLAG_MASK))
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "VDROP_OVP_FLAG =0x%02X\n",
            !!(flag & BQ2597X_VDROP_OVP_FLAG_MASK));

    /*read to clear alarm flag*/
    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_0E, &flag);
    if (!ret && flag)
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "INT_FLAG =0x%02X\n", flag);

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_0D, &stat);
    if (!ret && stat != pDevice->prev_alarm) {
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "INT_STAT = 0X%02x\n", stat);
        pDevice->prev_alarm = stat;
        pDevice->bat_ovp_alarm = !!(stat & BAT_OVP_ALARM);
        pDevice->bat_ocp_alarm = !!(stat & BAT_OCP_ALARM);
        pDevice->bus_ovp_alarm = !!(stat & BUS_OVP_ALARM);
        pDevice->bus_ocp_alarm = !!(stat & BUS_OCP_ALARM);
        pDevice->batt_present = !!(stat & VBAT_INSERT);
        pDevice->vbus_present = !!(stat & VBUS_INSERT);
        pDevice->bat_ucp_alarm = !!(stat & BAT_UCP_ALARM);
    }


    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_08, &stat);
    if (!ret && (stat & 0x50))
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "Reg[08]BUS_UCPOVP = 0x%02X\n", stat);

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_0A, &stat);
    if (!ret && (stat & 0x02))
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "Reg[0A]CONV_OCP = 0x%02X\n", stat);

}

static void bq2597x_check_fault_status(PBQ2597X_CONTEXT pDevice)
{
    int ret;
    UINT8 flag = 0;
    UINT8 stat = 0;
    bool changed = false;

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_10, &stat);
    if (!ret && stat)
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "FAULT_STAT = 0x%02X\n", stat);

    ret = bq2597x_read_reg(pDevice, BQ2597X_REG_11, &flag);
    if (!ret && flag)
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "FAULT_FLAG = 0x%02X\n", flag);

    if (!ret && flag != pDevice->prev_fault) {
        changed = true;
        pDevice->prev_fault = flag;
        pDevice->bat_ovp_fault = !!(flag & BAT_OVP_FAULT);
        pDevice->bat_ocp_fault = !!(flag & BAT_OCP_FAULT);
        pDevice->bus_ovp_fault = !!(flag & BUS_OVP_FAULT);
        pDevice->bus_ocp_fault = !!(flag & BUS_OCP_FAULT);
        pDevice->bat_therm_fault = !!(flag & TS_BAT_FAULT);
        pDevice->bus_therm_fault = !!(flag & TS_BUS_FAULT);

        pDevice->bat_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
        pDevice->bus_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
    }

}

static void bq2597x_dump_reg(PBQ2597X_CONTEXT pDevice)
{

    int ret;
    UINT8 val;
    UINT8 addr;
    if (bq_debug_flag) {
        for (addr = 0x00; addr <= 0x2B; addr++) {
            ret = bq2597x_read_reg(pDevice, addr, &val);
            if (!ret)
                TraceEvents(
                    TRACE_LEVEL_ERROR,
                    TRACE_DEVICE,
                    "Reg[%02X] = 0x%02X\n", addr, val);
        }
    }
}

static void bq2597x_charger_shutdown(PBQ2597X_CONTEXT pDevice)
{
    bq2597x_enable_adc(pDevice, false);
}

BOOLEAN
OnInterruptIsr(
    IN WDFINTERRUPT Interrupt,
    IN ULONG MessageID
)
{
    PBQ2597X_CONTEXT pDevice;
    BOOLEAN ret;
    unsigned char int_reg, int_msk;
    unsigned char masked_int;

    UNREFERENCED_PARAMETER(MessageID);

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Entering %!FUNC!");

    ret = false;
    pDevice = GetDeviceContext(WdfInterruptGetDevice(Interrupt));

    bq2597x_check_alarm_status(pDevice);
    bq2597x_check_fault_status(pDevice);

    bq2597x_charger_info(pDevice);
    bq2597x_dump_reg(pDevice);

    ret = true;
    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Exiting %!FUNC! - %!STATUS!",
        ret);
    return ret;
}

NTSTATUS
OnPrepareHardware(
    IN  WDFDEVICE     FxDevice,
    IN  WDFCMRESLIST  FxResourcesRaw,
    IN  WDFCMRESLIST  FxResourcesTranslated
)
/*++

Routine Description:

This routine caches the SPB resource connection ID.

Arguments:

FxDevice - a handle to the framework device object
FxResourcesRaw - list of translated hardware resources that
the PnP manager has assigned to the device
FxResourcesTranslated - list of raw hardware resources that
the PnP manager has assigned to the device

Return Value:

Status

--*/
{
    PBQ2597X_CONTEXT pDevice = GetDeviceContext(FxDevice);
    BOOLEAN fSpbResourceFound = FALSE;
    NTSTATUS status = STATUS_INSUFFICIENT_RESOURCES;

    UNREFERENCED_PARAMETER(FxResourcesRaw);

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Entering %!FUNC!");

    //
    // Parse the peripheral's resources.
    //

    ULONG resourceCount = WdfCmResourceListGetCount(FxResourcesTranslated);

    for (ULONG i = 0; i < resourceCount; i++)
    {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR pDescriptor;
        UCHAR Class;
        UCHAR Type;

        pDescriptor = WdfCmResourceListGetDescriptor(
            FxResourcesTranslated, i);

        switch (pDescriptor->Type)
        {
        case CmResourceTypeConnection:
            //
            // Look for I2C or SPI resource and save connection ID.
            //
            Class = pDescriptor->u.Connection.Class;
            Type = pDescriptor->u.Connection.Type;
            if (Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL &&
                Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)
            {
                if (fSpbResourceFound == FALSE)
                {
                    status = STATUS_SUCCESS;
                    pDevice->I2CContext.I2cResHubId.LowPart = pDescriptor->u.Connection.IdLowPart;
                    pDevice->I2CContext.I2cResHubId.HighPart = pDescriptor->u.Connection.IdHighPart;
                    fSpbResourceFound = TRUE;
                }
                else
                {
                }
            }
            break;
        default:
            //
            // Ignoring all other resource types.
            //
            break;
        }
    }

    //
    // An SPB resource is required.
    //

    if (fSpbResourceFound == FALSE)
    {
        status = STATUS_NOT_FOUND;
    }

    status = SpbTargetInitialize(FxDevice, &pDevice->I2CContext);

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Exiting %!FUNC! - %!STATUS!",
        status);
    return status;
}

NTSTATUS
OnReleaseHardware(
    IN  WDFDEVICE     FxDevice,
    IN  WDFCMRESLIST  FxResourcesTranslated
)
/*++

Routine Description:

Arguments:

FxDevice - a handle to the framework device object
FxResourcesTranslated - list of raw hardware resources that
the PnP manager has assigned to the device

Return Value:

Status

--*/
{
    PBQ2597X_CONTEXT pDevice = GetDeviceContext(FxDevice);
    NTSTATUS status = STATUS_SUCCESS;

    UNREFERENCED_PARAMETER(FxResourcesTranslated);

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Entering %!FUNC!");

    bq2597x_enable_adc(pDevice, false);

    SpbTargetDeinitialize(FxDevice, &pDevice->I2CContext);

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Exiting %!FUNC! - %!STATUS!",
        status);
    return status;
}

static void determine_initial_status(PBQ2597X_CONTEXT pDevice)
{
	if (pDevice->InterruptObject)
        OnInterruptIsr(pDevice->InterruptObject, 0);
}

NTSTATUS
OnD0Entry(
    IN  WDFDEVICE               FxDevice,
    IN  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

Routine Description:

This routine allocates objects needed by the driver.

Arguments:

FxDevice - a handle to the framework device object
FxPreviousState - previous power state

Return Value:

Status

--*/
{
    UNREFERENCED_PARAMETER(FxPreviousState);

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Entering %!FUNC!");

    PBQ2597X_CONTEXT pDevice = GetDeviceContext(FxDevice);
    NTSTATUS status = STATUS_SUCCESS;

    //start charger
    status = bq2597x_detect_device(pDevice);
    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "Failed to detect bq25970");
        return STATUS_IO_DEVICE_ERROR;
    }
    else {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "Device id=0x%x", status);
    }

    status = bq2597x_init_device(pDevice);
    if (status) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE,
            "Failed to init device");
        return STATUS_IO_DEVICE_ERROR;
    }

    status = WdfWaitLockCreate(
		WDF_NO_OBJECT_ATTRIBUTES,
		&pDevice->DataLock);

	if (!NT_SUCCESS(status))
	{
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "Error creating Data Waitlock - %!STATUS!",
			status);
		goto exit;
	}

    determine_initial_status(pDevice);

    pDevice->volt_qual = true;
    unsigned int shit = 0;

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "charge reg = %d", bq2597x_read_reg(pDevice, BQ2597X_REG_0C, &shit));

    if (pDevice->volt_qual) {
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "start charging on init\n");
        bq2597x_enable_charge(pDevice, true);
    }

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "charge reg = %d", bq2597x_read_reg(pDevice, BQ2597X_REG_0C, &shit));

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Exiting %!FUNC! - %!STATUS!",
        status);

exit:
    return status;
}

NTSTATUS
OnD0Exit(
    IN  WDFDEVICE               FxDevice,
    IN  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

Routine Description:

This routine destroys objects needed by the driver.

Arguments:

FxDevice - a handle to the framework device object
FxPreviousState - previous power state

Return Value:

Status

--*/
{
    UNREFERENCED_PARAMETER(FxPreviousState);

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Entering %!FUNC!");

    PBQ2597X_CONTEXT pDevice = GetDeviceContext(FxDevice);
    NTSTATUS status = STATUS_SUCCESS;

    bq2597x_charger_shutdown(pDevice);

    if (pDevice->DataLock != NULL)
	{
		WdfObjectDelete(pDevice->DataLock);
	}

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Exiting %!FUNC! - %!STATUS!",
        status);
    return status;
}

NTSTATUS
Bq2597xEvtDeviceAdd(
    IN WDFDRIVER       Driver,
    IN PWDFDEVICE_INIT DeviceInit
)
{
    NTSTATUS                      status = STATUS_SUCCESS;
    WDF_IO_QUEUE_CONFIG           queueConfig;
    WDF_INTERRUPT_CONFIG          interruptConfig;
    WDF_OBJECT_ATTRIBUTES         attributes;
    WDFDEVICE                     device;
    WDFQUEUE                      queue;
    PBQ2597X_CONTEXT              devContext;

    UNREFERENCED_PARAMETER(Driver);

    PAGED_CODE();

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Entering %!FUNC!");

    {
        WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;
        WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);

        pnpCallbacks.EvtDevicePrepareHardware = OnPrepareHardware;
        pnpCallbacks.EvtDeviceReleaseHardware = OnReleaseHardware;
        pnpCallbacks.EvtDeviceD0Entry = OnD0Entry;
        pnpCallbacks.EvtDeviceD0Exit = OnD0Exit;

        WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);
    }

    //
    // Setup the device context
    //

    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, BQ2597X_CONTEXT);

    //
    // Create a framework device object.This call will in turn create
    // a WDM device object, attach to the lower stack, and set the
    // appropriate flags and attributes.
    //

    status = WdfDeviceCreate(&DeviceInit, &attributes, &device);

    if (!NT_SUCCESS(status))
    {
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "WdfDeviceCreate failed with status code 0x%x\n", status);

        return status;
    }

    {
        WDF_DEVICE_STATE deviceState;
        WDF_DEVICE_STATE_INIT(&deviceState);

        deviceState.NotDisableable = WdfFalse;
        WdfDeviceSetDeviceState(device, &deviceState);
    }

    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&queueConfig, WdfIoQueueDispatchParallel);

    queueConfig.EvtIoInternalDeviceControl = Bq2597xEvtInternalDeviceControl;
    queueConfig.PowerManaged = WdfFalse;

    status = WdfIoQueueCreate(device,
        &queueConfig,
        WDF_NO_OBJECT_ATTRIBUTES,
        &queue
    );

    if (!NT_SUCCESS(status))
    {
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "WdfIoQueueCreate failed 0x%x\n", status);

        return status;
    }

    //
    // Create manual I/O queue to take care of hid report read requests
    //

    devContext = GetDeviceContext(device);

    devContext->FxDevice = device;

    WDF_IO_QUEUE_CONFIG_INIT(&queueConfig, WdfIoQueueDispatchManual);

    queueConfig.PowerManaged = WdfFalse;

    status = WdfIoQueueCreate(device,
        &queueConfig,
        WDF_NO_OBJECT_ATTRIBUTES,
        &devContext->ReportQueue
    );

    if (!NT_SUCCESS(status))
    {
        TraceEvents(
            TRACE_LEVEL_ERROR,
            TRACE_DEVICE,
            "WdfIoQueueCreate failed 0x%x\n", status);

        return status;
    }

    //
    // Create an interrupt object for hardware notifications
    //
    WDF_INTERRUPT_CONFIG_INIT(
        &interruptConfig,
        OnInterruptIsr,
        NULL);
    interruptConfig.PassiveHandling = TRUE;

    status = WdfInterruptCreate(
        device,
        &interruptConfig,
        WDF_NO_OBJECT_ATTRIBUTES,
        &devContext->InterruptObject);

    TraceEvents(
        TRACE_LEVEL_ERROR,
        TRACE_DEVICE,
        "Exiting %!FUNC! - %!STATUS!",
        status);
    return status;
}

VOID
Bq2597xEvtInternalDeviceControl(
    IN WDFQUEUE     Queue,
    IN WDFREQUEST   Request,
    IN size_t       OutputBufferLength,
    IN size_t       InputBufferLength,
    IN ULONG        IoControlCode
)
{
    NTSTATUS            status = STATUS_SUCCESS;
    WDFDEVICE           device;
    PBQ2597X_CONTEXT     devContext;

    UNREFERENCED_PARAMETER(OutputBufferLength);
    UNREFERENCED_PARAMETER(InputBufferLength);

    device = WdfIoQueueGetDevice(Queue);
    devContext = GetDeviceContext(device);

    switch (IoControlCode)
    {
    default:
        status = STATUS_NOT_SUPPORTED;
        break;
    }

    WdfRequestComplete(Request, status);

    return;
}