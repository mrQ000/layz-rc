/**
 * rapidM2M IoT-Box specific include file
 */
#pragma library IoTbox

/**
 * <summary>
 *   Available UI Channels.
 * </summary>
 */
const
{
  UI_CHANNEL1 = 0,
  UI_CHANNEL2,
  UI_NUM_CHANNELS,
};

/**
 * <summary>
 *   Sys info structure.
 * </summary>
 * <param name="VBat">
 *   Battery Voltage [mV]
 * </param>
 * <param name="VUsb">
 *   USB voltage [mV]
 * </param>
 * <param name="Temp">
 *   temperature [0,1°C]
 * </param>
 */
#define TIoTbox_SysValue[.VBat, .VUsb, .Temp]

/**
 * <summary>
 *   Led Modes.
 * </summary>
 */
const
{
  LED_MODE_INTERNAL = 0, // led's are controlled by FW
  LED_MODE_SCRIPT,       // led's are controlled by Script
};

/**
 * <summary>
 *   Retrieve sys values of device.
 * </summary>
 *
 * <param name="values">
 *   Structure where sys values are filled in.
 * </param>
 * <param name="len">
 *   length of values structure, max size according to TIoTbox_SysValue
 * </param>
 *
 * <returns>
 *   Returns:
 *   <ul>
 *     <li><em>used size of structure</em></li>
 *     <li><em>ERROR</em> - if address/length pair for structure is invalid or a sys value is invalid.</li>
 *   </ul>
 * </returns>
 */
native IoTbox_GetSysValues(values[TIoTbox_SysValue], len=sizeof values);

/**
 * <summary>
 *   Init Led usage.
 * </summary>
 *
 * <param name="mode">
 *   Led mode (see Led Modes).
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, <em>ERROR</em> in case of invalid parameters.
 * </returns>
 */
native Led_Init(mode);

/**
 * <summary>
 *   Close Led usage (neither FW nor Script control led's).
 * </summary>
 *
 * <returns>
 *   <em>OK</em>.
 * </returns>
 */
native Led_Close();

/**
 * <summary>
 *   Turn on led.
 * </summary>
 *
 * <param name="green">
 *   Green led on if true.
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, <em>ERROR</em> in case of invalid parameters.
 * </returns>
 */
native Led_On(color);

/**
 * <summary>
 *   Turn off led.
 * </summary>
 *
 * <param name="green">
 *   Green led off if true.
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, <em>ERROR</em> in case of invalid parameters.
 * </returns>
 */
native Led_Off();

/**
 * <summary>
 *   Blink with led (500ms On, 500ms Off).
 * </summary>
 *
 * <param name="green">
 *   Green led blink count or forever if 0.
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, <em>ERROR</em> in case of invalid parameters.
 * </returns>
 */
native Led_Blink(count, color);

/**
 * <summary>
 *   Flash with led (500ms period, single flash On).
 * </summary>
 *
 * <param name="green">
 *   Green led flash count or forever if 0.
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, <em>ERROR</em> in case of invalid parameters.
 * </returns>
 */
native Led_Flash(count, color);

/**
 * <summary>
 *   Flicker with led
 * </summary>
 *
 * <param name="green">
 *   Green led flicker count or forever if 0 .
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, <em>ERROR</em> in case of invalid parameters.
 * </returns>
 */
native Led_Flicker(count, color);

/**
 * <summary>
 *   Switch Modes.
 * </summary>
 */
const
{
  SWITCH_MODE_INTERNAL = 0, // switch is controlled by FW
  SWITCH_MODE_SCRIPT,       // switch is controlled by Script
};

/**
 * <summary>
 *   Init and configure switch interface.
 * </summary>
 *
 * <param name="mode">
 *   see Switch Modes.
 * </param>
 * <param name="funcidx">
 *   Index of callback function if script mode. Function is called on switch state change.
 *   Has to be of type: public func(key);
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, otherwise <em>ERROR</em>.
 * </returns>
 */
native Switch_Init(mode, funcidx=-1);

/**
 * <summary>
 *   Close and deactivate switch interface.
 * </summary>
 *
 * <returns>
 *   <em>OK</em> if successful, otherwise <em>ERROR</em>.
 * </returns>
 */
native Switch_Close();

/**
 * <summary>
 *   Identification structure of BLE chip.
 * </summary>
 * <param name="FWVersion">
 *   Firmware Version of BLE chip
 * </param>
 */
#define TBLE_Id[.FWVersion]

/**
 * <summary>
 *   Retrieve identification of BLE chip.
 * </summary>
 *
 * <param name="id">
 *   Structure where identification is filled in.
 * </param>
 * <param name="len">
 *   length of id Structure, max size according to TBLEGW_Id
 * </param>
 *
 * <returns>
 *   Returns:
 *   <ul>
 *     <li>number of filled cells for id struct</li>
 *     <li><em>ERROR</em> - if address/length pair for id struct is invalid.</li>
 *   </ul>
 * </returns>
 */
native BLE_GetId(id[TBLE_Id], len=sizeof id);

#define TBLE_Scan[.addr_type, .addr{6}, .rssi, .name{32+1}, .msd_len, .msd{32}]
#define TBLE_Notify[.handle, .data_len, .data{32}]
#define TBLE_Read[.handle, .offset, .data_len, .data{32}]

/**
 * <summary>
 *   BLE states.
 * </summary>
 */
const
{
  BLE_STATE_OFF = 0,
  BLE_STATE_INIT,
  BLE_STATE_READY,
  BLE_STATE_BUSY
};

/**
 * <summary>
 *   BLE events.
 * </summary>
 */
const
{
  BLE_EVENT_SCAN = 0,
  BLE_EVENT_SCAN_RSP,
  BLE_EVENT_NOTIFY,
  BLE_EVENT_READ,
};

/**
 * <summary>
 *   Init and configure BLE interface.
 * </summary>
 *
 * <param name="funcidx">
 *   Index of event callback function. Function is called on ble events.
 *   Has to be of type: public func(event, connhandle, const data{}, len);
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, otherwise <em>ERROR</em>.
 * </returns>
 */
native BLE_Init(funcidx);

/**
 * <summary>
 *   Close and deactivate BLE interface.
 * </summary>
 *
 * <returns>
 *   <em>OK</em> if successful, otherwise <em>ERROR</em>.
 * </returns>
 */
native BLE_Close();

/**
 * <summary>
 *   Reset BLE interface.
 * </summary>
 *
 * <returns>
 *   <em>OK</em> if successful, otherwise <em>ERROR</em>.
 * </returns>
 */
native BLE_Reset();

/**
 * <summary>
 *   Get BLE interface state.
 * </summary>
 *
 * <returns>
 *   State of BLE interface (see BLE states).
 * </returns>
 */
native BLE_GetState();

/**
 * <summary>
 *   Start scan
 * </summary>
 *
 * <param name="time">
 *   Time to scan in [s].
 * </param>
 *
 * <param name="flags">
 *   Flags for scanning.
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, otherwise <em>ERROR</em>.
 * </returns>
 */
native BLE_Scan(time = 10, flags = 0);

/**
 * <summary>
 *   Connect to BLE device.
 * </summary>
 *
 * <param name="addr">
 *   BLE address of device.
 * </param>
 *
 * <param name="itv">
 *   Connection interval in [ms] (valid 8..1000).
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, otherwise <em>ERROR</em>.
 * </returns>
 */
native BLE_Connect(addr{6}, itv = -1);

/**
 * <summary>
 *   Disconnect from BLE device (device must be connected).
 * </summary>
 *
 * <param name="connhandle">
 *   Connection handle (reserved for future use, set to 0).
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful
 *   <em>ERROR</em> if error
 *   <em>ERROR-1</em> if BLE device isn't connected.
 * </returns>
 */
native BLE_Disconnect(connhandle = 0);

/**
 * <summary>
 *   Get connection state.
 * </summary>
 *
 * <param name="connhandle">
 *   Connection handle (reserved for future use, set to 0).
 * </param>
 *
 * <returns>
 *   Connection state (0 = disconnected, >0 = connected).
 * </returns>
 */
native BLE_GetConnState(connhandle = 0);

/**
 * <summary>
 *   Write to BLE device (device must be connected).
 * </summary>
 *
 * <param name="connhandle">
 *   Connection handle (reserved for future use, set to 0).
 * </param>
 *
 * <param name="handle">
 *   Handle.
 * </param>
 *
 * <param name="data">
 *   Data.
 * </param>
 *
 * <param name="size">
 *   Length of data.
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful
 *   <em>ERROR</em> if error
 *   <em>ERROR-1</em> if BLE device isn't connected.
 * </returns>
 */
native BLE_Write(connhandle = 0, handle, const data{}, size);

/**
 * <summary>
 *   Read from BLE device (device must be connected).
 * </summary>
 *
 * <param name="connhandle">
 *   Connection handle (reserved for future use, set to 0).
 * </param>
 *
 * <param name="handle">
 *   Handle.
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful
 *   <em>ERROR</em> if error
 *   <em>ERROR-1</em> if BLE device isn't connected.
 * </returns>
 */
native BLE_Read(connhandle = 0, handle);

/**
 * <summary>
 *   Change connection intervall (device must be connected).
 * </summary>
 *
 * <param name="connhandle">
 *   Connection handle (reserved for future use, set to 0).
 * </param>
 *
 * <param name="conitv">
 *   Connection interval in [ms] (valid 8..1000).
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful
 *   <em>ERROR</em> if error
 *   <em>ERROR-1</em> if BLE device isn't connected.
 * </returns>
 */
native BLE_ChgConItv(connhandle = 0, conitv);




/**
 * UI channel module (UI = universal input)
 *
 * Includes functions for controlling UI channel modes, sample rate and to get actual value or
 * reset counter value of UI channel.
 */
#pragma library UI

/**
 * <summary>
 *   Measurement modes of UI channels.
 * </summary>
 */
const
{
  UI_CHT_SI_NONE = 0,   ///< channel off
  UI_CHT_SI_DIGITAL,    ///< digital
  UI_CHT_SI_DCTR,       ///< counter
  UI_CHT_SI_DFREQ,      ///< frequency
  UI_CHT_SI_DPWM,       ///< pulse width modulation
  UI_CHT_SI_A020MA,     ///< 0..20mA
  UI_CHT_SI_A002V,      ///< 0..2V
  UI_CHT_SI_A010V,      ///< 0..10V
  UI_CHT_SI_DIRECT,     ///< direct
};

/**
 * <summary>
 *   Sample rates for analog (0..20mA, 0..2V, 0..10V, direct)
 *   and digital measurement in [Hz].
 * </summary>
 *
 * <note>
 *   Higher sample rates result in higher current consumption!
 * </note>
 */
const
{
  UI_SAMPLE_RATE_2   =   2,
  UI_SAMPLE_RATE_4   =   4,
  UI_SAMPLE_RATE_8   =   8,
  UI_SAMPLE_RATE_16  =  16,
  UI_SAMPLE_RATE_32  =  32,
  UI_SAMPLE_RATE_64  =  64,
  UI_SAMPLE_RATE_128 = 128,
};

/**
 * <summary>
 *   Init an UI channel, configure channel mode and filter time.
 * </summary>
 *
 * <param name="channel">
 *   UI channel to configure (see device specific include file).
 * </param>
 * <param name="mode">
 *   Mode of channel (see UI_CHT_... channel modes).
 * </param>
 * <param name="filtertime">
 *   Filtertime for channel in [ms].
 *   Used for fast averaging of analog measurements (0..20mA, 0..2V, 0..10V, direct)
 *   or debouncing of digital value.
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, <em>ERROR</em> in case of invalid parameters, 
 *   <em>ERROR_NOT_SUPPORTED</em> in case mode is not supported on this channel.
 * </returns>
 *
 * <note>
 *   Each configured channel increases the current consumption!
 * </note>
 */
native UI_Init(channel, mode, filtertime);

/**
 * <summary>
 *   Close and deactivate an UI channel.
 * </summary>
 *
 * <param name="channel">
 *   UI channel to close.
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, <em>ERROR</em> in case of invalid parameter.
 * </returns>
 */
native UI_Close(channel);

/**
 * <summary>
 *   Get actual value of an UI channel.
 * </summary>
 *
 * <param name="channel">
 *   UI Channel to get value from.
 * </param>
 * <param name="value">
 *   Returns the value of the UI channel (channel mode dependant).
 *     UI_CHT_SI_DIGITAL  [0/1]
 *     UI_CHT_SI_DCTR     [-]
 *     UI_CHT_SI_DFREQ    [Hz]
 *     UI_CHT_SI_DPWM     [%]
 *     UI_CHT_SI_A020MA   [uA]
 *     UI_CHT_SI_A002V    [mV]
 *     UI_CHT_SI_A010V    [mV]
 *     UI_CHT_SI_DIRECT   [mV]
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, <em>ERROR</em> in case of invalid parameters.
 * </returns>
 */
native UI_GetValue(channel, &value=0);

/**
 * <summary>
 *   Set sample rate of measurement.
 * </summary>
 *
 * <param name="samplerate">
 *   Samplerate in [Hz] (only UI_SAMPLE_RATE_... values allowed!).
 *   Default samplerate is 16Hz.
 * </param>
 *
 * <returns>
 *   <em>OK</em> if successful, <em>ERROR</em> in case of invalid parameter.
 * </returns>
 *
 * <note>
 *   Higher sample rates result in higher current consumption!
 * </note>
 */
native UI_SetSampleRate(samplerate);

/**
 * <summary>
 *   Reset counters and get value.
 * </summary>
 *
 * <param name="channel">
 *   UI Channel to get value and reset counter.
 * </param>
 *
 * <returns>
 *   Count of channel, <em>ERROR</em> in case of invalid parameter.
 * </returns>
 */
native UI_ResetCounter(channel);

