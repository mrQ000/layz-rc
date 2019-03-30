/*
*/

#define S32_MIN 0x80000000
#define S32_MAX 0x7fffffff
#define now() rM2M_GetTime()
#define abs(%1) ((%1)<0?-(%1):(%1))



#define FALLBACK_TARGET 8   // if no schedule defined, fallback to this

#define BLOWER_TIMEOUT_S (  15*60)
#define SILENT_TIMEOUT_S (3*60*60)

#define UPLINK_NOK_AUTORETRY_S  (60*60) // seconds to wait until uplink forces recovery upon on-going error/disc status
#define LIVE_HOLD_S         90          // seconds to hold live mode after receiving a live trigger
#define TREND_INTERVAL_S  (5*60)
#define HEALTH_INTERVAL_S (3*60*60)    // [s] !!! recording is skipped if no faults counted !!!


#define FAST_TASK_MS            500    // [ms] task interval of low level control
#define TW_AVG_MS             10000    // [ms] small signal filter, min. stable time to accept new value

#define WP_OFF_2_DELAY_MS      3000    // [ms] waterpump off, step 2 delay
#define HEATER_ON_1_DELAY_MS   20000   // [ms] delay for turning on 1st heater element (orig: 2s)
                                       // ^- this lets the pump run a bit before deciding to heat
                                       // because while pump is stopped, the water at the tw sensor
                                       // is usally much colder as those in the pool
#define HEATER_ON_2_DELAY_MS   30000   // [ms] delay for turning on 2nd heater element (orig: 30s)
#define HEATER_OFF_WP_DELAY_MS 40000   // [ms] min. runtime of waterpump after heater turned off



#include "~auto.inc"
#define PIN_COUNT 6
#include "lib-iotbox"
#include "lib-xuino"

/*************************************************************************************************
  IO LAYOUT
*************************************************************************************************/
#define LIGHT_R 1          // gpio 2
#define LIGHT_G 3          // gpio 4
#define LIGHT_B 4          // gpio 5

#define BTN_LOCAL 5        // gpio 6

#define IOC_EMU 0
#define IOC_UART_toDSP 0   // uart 1 used by IOC emu, connected do physical DSP
#define IOC_layz layz0

#define DSP_EMU 1
#define DSP_UART_toIOC 1   // uart 2 used by DSP emu, connected to physical IOC
#define DSP_layz layz1

#include "app-io"
#include "lib-rm-dde.inc"
#include "lib-rm-uplink.inc"
#include "~auto-dde.inc"

// stamp when device powered on / booted
new tboot_;

// [s] remaining in live mode (e.g. /w fast status updates sent to backend)
new islive_;


#include "app-dde"
#include "app-layz"
#include "app-emu-ioc"
#include "app-emu-dsp"


_setLight( rgb, mode=LED_STATIC){
  setLight( rgb, mode);
  setSysLed( rgb, mode);
}

/*************************************************************************************************
  COMMANDs FOR LOCAL and/or REMOTE USE
*************************************************************************************************/

// stop blower automatically after a certain time
new blower_tmr_= 0;
timeout_blower_1s(){
  if (!blower_tmr_) return;   // timeout already halted -> done
  if (--blower_tmr_) return;  // timeout still running -> done

  // timeout just expired -> stop silent mode!
  if (dde_status.blower_mode) cmd_toggle_blower( "timeout",0);
}
cmd_toggle_blower( const src{50}, new_state=-1) {
  if (new_state<0) new_state= !dde_status.blower_mode;

  blower_tmr_= new_state ? BLOWER_TIMEOUT_S : 0;  // restart / stop timeout

  if (dde_status_set( 117, dde_status.blower_mode, new_state,
        dde_status.blower_mode ? "blower-on" : "blower-off", src)) {

    dsp_out.air= dde_status.blower_mode;  // todo this is a bit redundant...
  }
}

// stop silent mode automatically after a certain time
new silent_tmr_= 0;
timeout_silent_1s(){
  if (!silent_tmr_) return;   // timeout already halted -> done
  if (--silent_tmr_) return;  // timeout still running -> done

  // timeout just expired -> stop silent mode!
  if (dde_status.silent_mode) cmd_toggle_silent("timeout",0);
}
cmd_toggle_silent( const src{50}, new_state=-1) {
  if (new_state<0) new_state= !dde_status.silent_mode;

  silent_tmr_= new_state ? SILENT_TIMEOUT_S : 0;  // restart / stop timeout

  dde_status_set( 116, dde_status.silent_mode, new_state,
        dde_status.silent_mode ? "silent-on" : "silent-off", src);
}

cmd_stats_reset( const src{50}){
  dde_event( "resetstats", src);

  dsp_stats_.ticks= 0;
  dsp_stats_.air= 0;
  dsp_stats_.wp= 0;
  dsp_stats_.htr= 0;

  dde_status.stats_Exx= 0;
  dde_status.stats_reset= now();
  dde_status.stats_energy= 0;
  dde_status.stats_blower= 0;
  dde_status.stats_heater= 0;
  dde_status.stats_pump= 0;
  dde_status.stats_twmin= S32_MAX;
  dde_status.stats_twmax= S32_MIN;

  dde_status_dirty(112);
}


/*************************************************************************************************
  LOCAL (MANUAL) CONTROL
*************************************************************************************************/
forward on_btnhold();
public on_btnhold(){
  cmd_toggle_blower( "local", 0);
  cmd_toggle_silent( "local", 1);
  _setLight( 0x202020);
}
public onButton( pressed, time_){

  // block the button for the first 5s after power on
  // to suppress unintended button presses
  if ((now() - tboot_) < 5) return;

  if (pressed) {
    setTimeout( "on_btnhold", 2000);
    _setLight( 0xffffff - dde_setup.light);
  }
  else {
    clearTimeout( "on_btnhold");
    if (time_<1000) {
      if (dde_status.silent_mode)
        cmd_toggle_silent( "local", 0);
      else
        cmd_toggle_blower( "local");
    }
    _setLight( dde_setup.light);
  }
}

/*************************************************************************************************
  LOCAL SIGNALLING
  boot waiting			- weiss flackern
  Exx pending			- rot blinken
  boot connecting, but running	- glau flackern
  reconnecting long lasting fault - rot flackern
  sonst                           - farbe aus setup.light

  parallel
  key long ? silent on		- 1x blinken mit farbumkehr
  key short ? silent off		- 2x bit farbumkehr

*************************************************************************************************/
new opmode_= 0; // 0= power on, 1= waiting for cfg from BE, 2= ready, doing last inits, 3= running

signal_task_1s(){

  if (opmode_<=1)                     _setLight( 0xffffff, LED_FLICKER); // SIG_NOPROG;
  else if (dde_status.Exx)            _setLight( 0xff0000, LED_BLINK); // SIG_EXX;
  else {
    new upl= rM2M_TxGetStatus();
    if (upl & RM2M_TX_FAILED)         _setLight( 0xff0000, LED_FLICKER); // SIG_UPLFAULT;
    else if (!(upl & RM2M_TX_ACTIVE)) _setLight( 0x0000ff, LED_FLICKER); // SIG_UPLDISC;
    else                              _setLight( dde_setup.light); // SIG_OK;
  }
}

/*************************************************************************************************
  FAST TASK
*************************************************************************************************/
forward fast_task_500ms();
public fast_task_500ms(){
  dsp_emu_task_500ms();
  dsp_PHctrlr_500ms();
  dde_status_write_task_500ms();
}

/*************************************************************************************************
  MAIN TASK
*************************************************************************************************/

main_record_trend( stamp){

    new sv[TIoTbox_SysValue]; // [.VBat, .VUsb, .Temp]
    IoTbox_GetSysValues( sv);
    printf("Vbat=%dmV Vusb=%dmV Tsys=%d°C\n", sv.VBat, sv.VUsb, sv.Temp/10);

    // trigger "Ubat lo/hi" event
    static ubat_lohi_= 0;
    dde_event_lohi( "ubat", ubat_lohi_, sv.VBat, 3800, 4400);

    // trigger "Uusb lo/hi" event
    static uusb_lohi_= 0;
    dde_event_lohi( "uusb", uusb_lohi_, sv.VUsb, 4700, 5200);

    // trigger "Tsys lo/hi" event
    static tsys_lohi_= 0;
    dde_event_lohi( "tsys", tsys_lohi_, sv.Temp, 50, 5000); // 0,1°C

    dde_trend_append( stamp,
      dsp_mon.Exx,
      dsp_mon.tw_acc,
      dsp_mon.tw_cnt,
      dsp_mon.ticks,
      dsp_mon.air,
      dsp_mon.wp,
      (dsp_mon.htr1+dsp_mon.htr2)/ 2,
      sv.VBat,
      sv.VUsb,
      sv.Temp/10
    );

    // reset period accumulators
    dsp_mon.Exx= 0;
    dsp_mon.tw_acc= 0;
    dsp_mon.tw_cnt= 0;

    dsp_mon.air  = 0;
    dsp_mon.wp   = 0;
    dsp_mon.htr1 = 0;
    dsp_mon.htr2 = 0;
    dsp_mon.ticks= 0;
}

forward main_task_1s();
public main_task_1s(){
  new t;

  timeout_silent_1s();
  timeout_blower_1s();

  /* ---
    LIVE MODE
  --- */
  if (islive_) {
    --islive_;
    if (!islive_) dde_event( "live","ended");
  }

  /* ---
    TREND RECORDING INTERVAL
  --- */
  static trend_ticks_= 0;
  t= now() / TREND_INTERVAL_S;
  if (t > trend_ticks_) {
    trend_ticks_= t;
    main_record_trend((t-1)*TREND_INTERVAL_S);

    if(status_dirty_>S32_MAX/2) _dde_status_write();
  }

  /* ---
    HEALTH INTERVAL
  --- */
  static health_ticks_= 0;
  t= now() / HEALTH_INTERVAL_S;
  if (t > health_ticks_) {
    health_ticks_= t;
    layzCheckHealth( DSP_EMU);
    layzCheckHealth( IOC_EMU);
  }

  /* ---
    PHY DISPLAY UPDATER - show current target and current water temp. on phy display
  --- */
  static screen_= 0;
  screen_= ++screen_ & 3;
  // if Exx pending, show error code
  // - but as "temp." value and NOT as "Exx" to avoid that phy dsp enters blocking error operation!
  if (dsp_in.Exx) ioc_out.temp= dsp_in.Exx;
  // otherwise toggle between program | target | current temp.
  else if (screen_==0) ioc_out.temp = dde_status.prognum;
  else if (screen_==1) ioc_out.temp = dde_status.target;
  else ioc_out.temp = dsp_in.tw_avg;

  /* ---
    do local signalling
  --- */
  signal_task_1s();
}


_on_cmds() {

      new o[DDE_cmds];
      if (DDE_cmds_read( o)) return ERROR;

      if (dde_status_set( 113, dde_status.blower_trg, o.blower_trg)){
        cmd_toggle_blower( "remote");
      }
      if (dde_status_set( 114, dde_status.silent_trg, o.silent_trg)){
        cmd_toggle_silent( "remote");
      }
      if (dde_status_set( 115, dde_status.resetstats_trg, o.resetstats_trg)){
        cmd_stats_reset( "remote");
      }
      if (dde_status_set( 111, dde_status.live_trg, o.live_trg)) {
        printf("DDE_CMDS liveTrig\n");
        if (!islive_) dde_event( "live","started");
        islive_= LIVE_HOLD_S;
      }
      return OK;
}
_on_setup(){
      new o[DDE_setup];
      if (DDE_setup_read( o)) return ERROR;

      if (dde_setup.light != o.light) {
        printf("DDE_SETUP light %06x\n", o.light);
        _setLight( o.light);
      }
      new progs_changed= 0;
      for( new j=0;j<sizeof dde_setup.prog_start;j++) {
        if ((dde_setup.prog_start[j]  != o.prog_start[j]) ||
            (dde_setup.prog_end[j]    != o.prog_end[j])   ||
            (dde_setup.prog_target[j] != o.prog_target[j])) progs_changed |= (1<<j);
      }
      if (progs_changed) {
        printf("DDE_SETUP progs changed\n", progs_changed);
        for( new j=0; j<sizeof dde_setup.prog_start; j++) {
          new chgd= (progs_changed & (1<<j)) ? '*':' ';

          if (o.prog_start[j] && o.prog_end[j] && o.prog_target[j]>0) {
            new targ{10};
            if (o.prog_target[j] < 6) {
              targ= "pump";
            }
            else {
              sprintf( targ,_,"%2d°C", o.prog_target[j]);
            }
            new y,m,d,h,n,s;
            rM2M_GetTime( h,n,s, o.prog_start[j]);
            rM2M_GetDate( y,m,d, o.prog_start[j]);
            new dt= o.prog_end[j] - o.prog_start[j];
            printf("   %c [%d] %s from %02d-%02d-%02d %02d:%02d for %02d:%02d\n",
              chgd, j, targ, y,m,d,h,n, dt/3600, dt/60%60);
          }
          else {
            printf("   %c [%d] ---\n", chgd, j);
          }
        }
      }
      dde_setup= o;
      return OK;
}
_on_profile(){
      new o[DDE_profile];
      if (DDE_profile_read( o)) return ERROR;

      if ((dde_profile.price_per_kwh != o.price_per_kwh) ||
          (dde_profile.blower_load   != o.blower_load) ||
          (dde_profile.heater_load   != o.heater_load) ||
          (dde_profile.pump_load     != o.pump_load) ||
          (dde_profile.debug_flags   != o.debug_flags)) {

        printf("DDE_PROFILE changed %.2f/kwh blower %dW, heater %dW, pump %dW, debug=0x%04x\n",
          o.price_per_kwh, o.blower_load, o.heater_load, o.pump_load, o.debug_flags);

        layz_framedump= !!(o.debug_flags & 0x0001);
        layz_bridge   = !!(o.debug_flags & 0x0002);
      }
      dde_profile= o;
      return OK;
}

// cx:s32 - 0-9 = container configX
public onUplinkReceived( const cx){
  switch( cx) {
    case DDE_cmds_id: return _on_cmds();
    case DDE_setup_id: return _on_setup();
    case DDE_profile_id: return _on_profile();
    default: {
      printf("! onUplinkReceived - unknown config%d\n", cx); // todo dde_event (einmalig)???
    }
  }
  return ERROR;
}

public onUplinkReady(){
  opmode_= 2;
  dde_event( "boot","init-ready");

  islive_= LIVE_HOLD_S;
  uplinkMode( UPLINK_ONLINE); // UPLINK_TRIG, _WAKEUP, _ONLINE | UPLINK_NOPOS

  // layz_bridge= 1;
  layzInit( DSP_UART_toIOC, "dsp", LAYZ_BOF_TX_DSP, LAYZ_BOF_TX_IOC, "on_rx_dsp", "on_txto_dsp", 250);
  layzInit( IOC_UART_toDSP, "ioc", LAYZ_BOF_TX_IOC, LAYZ_BOF_TX_DSP, "on_rx_ioc", "on_txto_ioc", 1700);

  setInterval( "fast_task_500ms", FAST_TASK_MS);
  setInterval( "main_task_1s", 1000);

  opmode_= 3;
  dde_event( "boot","running");
  _setLight( dde_setup.light);
}

public onUplinkWaiting(){
  dde_event( "boot","init-waiting");
  opmode_=1;
}

/*
  monitor uplink connection and force recovery if it hangs for too long time in
  error or disconnected state during ONLINE mode
  (which is active during boot and optionally during regular operation)
*/
forward uplink_task_1s();
public uplink_task_1s(){

  static tnok_= 0;    // number of seconds in not-ok (error,disc) status

  // ONLINE mode NOT active -> done!
  if (UPLINK_ONLINE != uplinkMode()) {
    tnok_= 0;
    return;
  }

  // uplink is OK -> done!
  new upl= rM2M_TxGetStatus();
  new nok= (upl & RM2M_TX_FAILED) || !(upl & RM2M_TX_ACTIVE);
  if (!nok) {
    tnok_= 0;
    return;
  }

  // uplink not timedout yet -> done (and accumulate time) !
  tnok_++;
  if (tnok_ < UPLINK_NOK_AUTORETRY_S) {
    return;
  }

  // uplink timed out -> try to recover!
  dde_event( "upl","recover");
  flushUplink();
  tnok_= 0;
}


static consolebuffer{4096};

main(){
  opmode_= 0;
  tboot_= now();

  setbuf( consolebuffer, 4096);
  //  rM2M_TxSetMode( UPLINK_WAKEUP); // force disconnect

  dde_event( "boot","========================= power-on ===");

  ioInit();

  // init/restore UP containers
  DDE_status_read( dde_status);
  dde_status.Exx= 0;            // reset current Exx to get raise of Exx into event log
  dde_status.tw = -999;         // reset to grant at least one "tw" event log after boot
  cmd_stats_reset( "boot");     // reset stats on every boot 'cause otherwise tick-cnt and time-span will not match anymore!


  // connect uplink and wait until DN containers available
  // proceeds with onUplinkReady (and eventually an intermediate onUplinkWaiting)
  initUplink();
  setInterval( "uplink_task_1s", 1000);

  dde_event( "boot","initializing");
}

