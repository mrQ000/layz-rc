/*************************************************************************************************
  LAYZ DISPLAY EMULATOR
*************************************************************************************************/

// current outputs of DSP-emu to be sent to phy-IOC (by default all off)
new dsp_out[
  .air,       // air blower on
  .wp1,       // water pump #1 ... turn wp OFF: clear wp2 -> wait 5500ms, clear wp1
  .wp2,       // water pump #2
  .wphtr,     // wp lock by heater ... heater OFF: wait xxxx ms -> clear wphtr
  .htr0,
  .htr1,
  .htr2
];


// last seen status reported by phy-IOC to DSP-emu
new dsp_in[
  .booted,   // ready flag from phy-IOC
  .Exx,      // Exx value  from phy-IOC
  .tw_raw,   // raw °C  from phy-IOC - may flicker a bit!!!    
  .tw_avg    // filtered value
];

new dsp_ctrl[
  .ph,         // pump/heater operating mode; 0=all off, 1=waterpump on, 2=heater+waterpump on
  .heater,     // ctrl micro-step
  .pump        // ctrl micro-step
               //   ^--- >0 ... turning on since N*100ms,
               //        <0 ... turning off since -N*100ms,
               //         0 ... initial state, treated as "turned off"
];



// buffer to aggregate trend values
new dsp_mon[

  .Exx,       // highest error number seen during this operation time-window

  // operation time monitor
  .air,       // number of ticks that blower was on during observation time-window
  .wp,        // -"- water pump -"- 
  .htr1,      // -"- heater 1 -"-
  .htr2,      // -"- heater 2 -"-
  .ticks,     // the observation time-window's current length e.g. total number of ticks

  // temp averager
  .tw_acc,
  .tw_cnt       // number of accumulated values in .tw_acc
]
// hires intermediate values to count values for status.stats_xxx
new dsp_stats_[
  // operation time monitor
  .ticks,
  .air,       // [ticks] number of ticks that blower was on
  .wp,        // [ticks] -"- water pump -"- 
  .htr        // [ticks] -"- heater -"-
]

// buffer used for change detection
new dsp_diff[
  .booted
];


// Exx changed -> record to event log
_dsp_set_Exx( exx) {
  dsp_in.Exx  = exx;
  dsp_mon.Exx = max( dsp_mon.Exx, exx);
  dde_status_set( 41, dde_status.stats_Exx, max( dde_status.stats_Exx, exx));

  if (dde_status_set( 119, dde_status.Exx, exx, "Exx", "E%02d from physical controller")) {
    // Exx raised -> trigger immediate emergency stop!!!
    if (exx) layzSend( DSP_EMU, 0, 0, 0);
    islive_= LIVE_HOLD_S;	// immediately report to backend
  }
}

const {
  PH_OFF = 0,
  PH_PUMP= 1,
  PH_HEAT= 2
}
setPHctrlr( ph){
  if (ph == dsp_ctrl.ph) return;

  new s{50};
  sprintf( s,_,"%d (%d)", ph, dsp_ctrl.ph);
  dde_event( "phctrl", s);

  dsp_ctrl.ph= ph;
  // ---
  // all off (heater+pump)
  // ---
  if (ph == PH_OFF) {

    dde_event( "phctrlr", "htr-off");
    dsp_ctrl.heater= -1;
    // turn htr OFF -> clear htr0/1/2 immediately
    dsp_out.htr0= false;
    dsp_out.htr1= false;
    dsp_out.htr2= false;
    // dsp_out.wphtr= false; - with delay!

    dde_event( "phctrlr", "wp-off");
    dsp_ctrl.pump= -1;
    // turn wp OFF -> clear wp2, wait 5500ms, clear wp1
    dsp_out.wp2= false;
    // todo - keine negative auswirkung bei sofortiger abschaltung feststellbar
    // todo - wp2 zeigt auch keine änderung beim stromverbrauch
    // dsp_out.wp1= false;
  }
  else { // PH_PUMP and/or PH_HEAT
    // ---
    // pump on
    // ---
    dde_event( "phctrlr", "wp-on");
    dsp_ctrl.pump= 1;
    // turn wp ON -> set wp1 && wp2 immediately
    dsp_out.wp1= true;
    dsp_out.wp2= true;

    // ---
    // heater on, too
    // ---
    if (ph == PH_HEAT) {
      dde_event( "phctrlr", "htr-on");
      dsp_ctrl.heater= 1;
      // turn htr ON -> set htr0, wait 2s, set htr1, wait 30s, set htr2
      dsp_out.htr0= true;
      dsp_out.htr1= false;
      dsp_out.htr2= false;
      dsp_out.wphtr= true;
    }
  }
}

/* ---
  PUMP/HEATER CONTROL TASK
--- */
dsp_PHctrlr_500ms() {

  #define SLICE_MS 500

  // turning WP OFF
  if (dsp_ctrl.pump < 0) {
    dsp_ctrl.pump= max( --dsp_ctrl.pump, -1000);  // limit ctr to avoid range overflow after ~2500 days!
    
    if (dsp_ctrl.pump < -WP_OFF_2_DELAY_MS/SLICE_MS) dsp_out.wp1= false;   // todo original 5.5s
  }
  
  // turning HEATER ON
  if (dsp_ctrl.heater > 0) {
    dsp_ctrl.heater= min( ++dsp_ctrl.heater, 1000);  // limit ctr to avoid range overflow after ~2500 days!
    
    if (dsp_ctrl.heater >= HEATER_ON_1_DELAY_MS/SLICE_MS) dsp_out.htr1= true;
    if (dsp_ctrl.heater >= HEATER_ON_2_DELAY_MS/SLICE_MS) dsp_out.htr2= true;  // original: 300
  }  

  // turning HEATER OFF
  if (dsp_ctrl.heater < 0) {
    dsp_ctrl.heater= max( --dsp_ctrl.heater, -1000);  // limit ctr to avoid range overflow after ~2500 days!
    
    if (dsp_ctrl.heater < -HEATER_OFF_WP_DELAY_MS/SLICE_MS) dsp_out.wphtr= false;
  }  
}


new dsp_txto_cnt_=0;

forward on_txto_dsp();
public on_txto_dsp(){
  // todo dde_event( "dsp-txto", "physical io controller connected?");

  // if itf is down for >3 cycles, then send controller to critical state
  if (++dsp_txto_cnt_ > 3) {
    _dsp_set_Exx( LAYZ_Exx_ITF_TIMEOUT);
    // ^-- automatically reset with ioc's Exx as soon as next frame comes in
  }
}


forward on_rx_dsp();
public on_rx_dsp(){

  new z[Tlayz];
  layz(z,DSP_EMU);
  layzDumpFrame( DSP_EMU, LAYZ_DUMP_RX, z.rxbuf);

  new tw= z.rxbuf{2};
  new Exx = z.rxbuf{3};
  new booted= z.rxbuf{4};
  
  // validate payload of rxframe (others already checked by layz_rx_char)
  // (e.g. check if unused bits are all zero)
  new ok= (tw < 50) ||  // temp °C - todo negative values should never occur?
          (Exx < 20) ||  // E00..E20
          ((booted & 0xfe) == 0x00);
  if (!ok) {
    DSP_layz.cnt_dubious++;
    return;
  }  

  dsp_txto_cnt_= 0;

  dsp_in.tw_raw= tw;
  dsp_in.booted= booted;

  _dsp_set_Exx( Exx);

  // booted flag changed -> record to event log
  if (dsp_diff.booted != dsp_in.booted) {
    dsp_diff.booted= dsp_in.booted;
    if (booted) dde_event( "booted", "ioc-phy");
    else        dde_event( "warn"  , "ioc-phy cleared booted flag");
  }


  /* todo rem
  // ---
  // short term smoothing for control
  // ---
  // - phase 1: average to increase resolution beynd 1°C
  new sum= tw;
  for( new i=1; i<sizeof dsp_in._tws; i++) {
    new t= dsp_in._tws[i-1];
    if (t==0) t= tw;         // _tws was all zero -> needs initialization!
    dsp_in._tws[i]= t;
    sum += t;
  }
  dsp_in._tws[0]= tw;
  // - phase 2: threshold to avoid nervos flickering
  if (abs(dsp_in._tws_sum - sum) > 2) {
    dsp_in._tws_sum= sum;
    dsp_in.tw_avg= fround( sum / sizeof dsp_in._tws);
  } */
/* todo rem  
  // - step 1: send new value to buffer queue
  for( new i=1; i<sizeof dsp_in._tws; i++) {
    new t= dsp_in._tws[i-1];
    dsp_in._tws[i]= t==0 ? tw : t;	// if _tws[x] is zero -> needs initialization!
  }
  dsp_in._tws[0]= tw;

  // - step 2a: accept new tw if new value represents a "jump" > 1°C
  new accept = abs(tw-dsp_in._tws[1]) > 1;

  // - otherwise step 2b: accept new tw if whole buffer contains same value
  if (!accept) {
	  for( new i=1; i<sizeof dsp_in._tws; i++) {
		accept= dsp_in._tws[i] == dsp_in._tws[i-1];
		if (!accept) break;
	  }
  }
  
  // - step 3: set accepted tw as "avg" value
  if (accept) {
	dsp_in.tw_avg= tw;
  } */
  
  
  // ---
  // apply small signal filter to eliminate flickering value
  // ---
  
  // count value repetations
  static tw_prevval_=-999;	// small signal filter - previous value; (-999 grants initialisation)
  static tw_prevcnt_=0;		// small signal filter - repeat count of prevval
  if (tw == tw_prevval_) {	// last known signal..
	tw_prevcnt_++;			// ..repeated!
  }
  else {					// ..not repeated!
	tw_prevval_= tw;
	tw_prevcnt_= 1;  
  }
  
  new tw_diff= abs( tw - dsp_in.tw_avg);
  
  // accept new tw if new value represents a "large signal jump" > 1°C
  new large_signal = tw_diff > 1;
  // otherwise: accept new tw if it is stable for >= TW_AVG_MS
  new small_signal = tw_diff > 0 && tw_prevcnt_ > TW_AVG_MS / FAST_TASK_MS;
  
  // - step 3: set accepted tw as "avg" value
  if (large_signal || small_signal) {
	dsp_in.tw_avg= tw;
  }
  
  

  // trigger immediate transmission when tw falls to (or below) 6°C 
  if (dde_status.tw>6 && dsp_in.tw_avg<=6){
	islive_= LIVE_HOLD_S;
  }
  // avg'd water temp. changed -> record to event log
  dde_status_set( 3, dde_status.tw, dsp_in.tw_avg, "tw");

  // todo one day we may have an air temp, too...
  dde_status.tair= -127;

  // long term min/max/avg for trend recording
  new twmin = min( tw, dde_status.stats_twmin);
  new twmax = max( tw, dde_status.stats_twmax);
  dde_status_set( 4, dde_status.stats_twmin, twmin);
  dde_status_set( 2, dde_status.stats_twmax, twmax);

  dsp_mon.tw_acc += tw;
  dsp_mon.tw_cnt++;

  // ---
  // run automatic waterpump/heater control
  // ---

  // get current target and log changes of target
  new targ= dde_setup_current_target();

  // compare target with current situation
  if (targ==0) setPHctrlr( PH_OFF);
  else if (targ < 6) setPHctrlr( PH_PUMP);
  else {
    if (targ > 40) targ = 40;
    if (dsp_in.tw_avg > targ) setPHctrlr( PH_OFF);
    if (dsp_in.tw_avg < targ) setPHctrlr( PH_HEAT);
  }
}

dsp_emu_task_500ms(){

    static Exx_= 0;

    new exx= dsp_in.Exx;

    // set into virtual E90 mode while ioc has not booted
    // this suppresses application of any prog 'cause turning on anything
    // during boot is rejected with Exx faults by the ioc
    if (!dsp_in.booted) exx= 90;

    if (!Exx_ && exx)      dde_event( "exx","enter emergency mode");
    else if (Exx_ && !exx) dde_event( "exx","leave emergency mode");
    Exx_= exx;

    new f2= 0x00;
    new f4= 0x00;
  
    if (exx) {
      // turn everything immediately off in case of Exx
    }
    else if (dsp_out.air) {
      f2 |= 0x20;
      // if airblower is on, waterpump and heater are forced off!
    }
    else {      
      if (dsp_out.htr0) f2 |= 0x01;
      if (dsp_out.htr1) f2 |= 0x02;
      if (dsp_out.htr2) f2 |= 0x08;

      // force waterpump to ON if heater is on
      new htron= dsp_out.wphtr || dsp_out.htr0 || dsp_out.htr1 || dsp_out.htr2;  // force pump on if htr is on!
      if (htron || dsp_out.wp1) f2 |= 0x10;
      if (htron || dsp_out.wp2) f4 |= 0x01;
    }  
    layzSend( DSP_EMU, f2, 0, f4);

    new air =  (f2 & 0x20) ? 1 : 0;
    new wp  = ((f2 & 0x10) || (f4 & 0x01)) ? 1 : 0;
    new htr1=  (f2 & 0x02) ? 1 : 0;
    new htr2=  (f2 & 0x08) ? 1 : 0;


    // current on/off monitor
    dde_status_set( 21, dde_status.blower_on, air);
    dde_status_set( 22, dde_status.pump_on  , wp);
    dde_status_set( 23, dde_status.heater_on, htr1+htr2);

    // current load monitor
    new load = 0;
    if (dde_status.blower_on) load+= dde_profile.blower_load;
    if (dde_status.pump_on  ) load+= dde_profile.pump_load;
    if (dde_status.heater_on) load+= dde_profile.heater_load;
    dde_status_set( 29, dde_status.load, load);

    // effective runtime monitor
    dsp_mon.ticks++;
    dsp_mon.air  += air;
    dsp_mon.wp   += wp;
    dsp_mon.htr1 += htr1;
    dsp_mon.htr2 += htr2;

    dsp_stats_.ticks++;
    dsp_stats_.air  += air;
    dsp_stats_.wp   += wp;
    dsp_stats_.htr  += (htr1||htr2) ? 1 : 0;

    // resetable stats monitor
    // todo dde_status_dirty in case of any change
    // todo ..may be not so important here due to very low change frequency ????
    new Float:fsecs= now() - dde_status.stats_reset;
    new Float:ftks = max( 1, dsp_stats_.ticks);
    new Float:secs_per_tick= fsecs / ftks;
    new air_secs= secs_per_tick * dsp_stats_.air; // ticks -> secs
    new wp_secs = secs_per_tick * dsp_stats_.wp;
    new htr_secs= secs_per_tick * dsp_stats_.htr;

    dde_status_set( 31, dde_status.stats_blower, air_secs*4/3600); // secs -> 1/4hrs
    dde_status_set( 32, dde_status.stats_heater, htr_secs*4/3600); // secs -> 1/4hrs
    dde_status_set( 33, dde_status.stats_pump  , wp_secs *4/3600); // secs -> 1/4hrs

    new en =
      dde_profile.blower_load * air_secs +
      dde_profile.pump_load   * wp_secs  +
      dde_profile.heater_load * htr_secs; // Ws

    en= en*2/3600/1000; // Ws -> 0,5kWh
    dde_status_set( 5, dde_status.stats_energy, en);
}
