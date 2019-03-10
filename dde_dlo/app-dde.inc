/*************************************************************************************************
  LAY-Z REMOTE CONTROL   -   APP's UPLINK DATA INTERFACE
*************************************************************************************************/

// append new event to event series
dde_event( const code{12}, const params{50}) {
  printf("DDE_EVENT %s - %s\n", code, params);
  DDE_events_write( _, code, params);
}

dde_event_lohi( const code{10}, &buf, const val, const vlo, const vhi) {
	new lohi= 0;
	if (val < vlo) lohi= 1;
	if (val > vhi) lohi= 2;
    
	new txt[]{}= ["ok","lo","hi"];
	if (buf != lohi) {
		buf = lohi;
		new s{50};
		sprintf( s,_,"%s %d", txt[lohi], val);
		dde_event( code, s);
    }
}



// current device profile
new dde_profile[DDE_profile];
  // .debug_flags
  //      0x0001 = dump raw layz frames whenever they change
  //      0x0002 = bridge mode

// current device status
new dde_status[DDE_status];
    // .target,        // current operating target; 0=all off, 1=waterpump on, >5=heater+waterpump on

// -> dde_status_dirty()
// 0=not dirty; >0 is dirty, do write in Xms; S32_MAX=dirty, do write at next trend recoring
new status_dirty_;

// record current dde_status (and send it to backend)
_dde_status_write(){
  status_dirty_= 0;
  printf("DDE_STATUS write\n");
  DDE_status_write( dde_status);
}

// periodically check if current status shell be recorded
dde_status_write_task_500ms(){
  if (!status_dirty_) return;

  status_dirty_ -= 500;
  if(status_dirty_<=0) _dde_status_write();
}

// dde_status has been modified - request deferred recording (delay depends on live mode)
dde_status_dirty( const prio_src){
  new delay;
  new pronto= prio_src >= 100;

  if (pronto) delay= 500
  else {
    if (islive_) delay= 3000
    else delay= S32_MAX;
  }
  status_dirty_= status_dirty_ ? min( status_dirty_, delay) : delay;

  printf("DDE_STATUS dirty %d\n", prio_src);
}

/**
 * check if a dde_status.xxx has changed, if so then
 * - set the prop
 * - signal dde_status dirty
 * - and append an event log (optional)
 *
 * @returns boolean - 1 if changed, 0 if unchanged
 */
dde_status_set( const prio_src, &ref, const val, const eventcode{12}="", const eventfmt{}=""){

  if (ref==val) return 0;       // abort if not changed

  if (eventcode{0}) {           // write event (optional)
    new s{50};
    if (eventfmt{0}) sprintf( s, _, eventfmt, val, ref);
    else             sprintf( s, _, "%d (%d)", val, ref);
    dde_event( eventcode, s);
  }
  
  ref= val;                     // update changes
  dde_status_dirty( prio_src);  // mark dde_status as dirty and set write request
  
  return 1;
}

// current setup
new dde_setup[DDE_setup];
  // .progs_target[10]    // 0=off, 1=pump only, >5=heater+pump, <0=prog disabled
  // .progs_start[10]     // 0=prog disabled, >0 = stamp
  // .progs_end[10]       // 0=prog disabled, >0 = stamp

/**
 * read current target from setup
 * implies automatic selection of prog
 * - the first which fits to current time / now
 * - FALLBACK_TARGET if no fitting prog found
 *
 * @returns target - 0=off, 1=pump only, >5=heater+pump
 */
dde_setup_current_target(){

	new prognum=-2; // -2 = "silent mode", -1=fallback, 1..10=regular prog
	new targ= 0;    // default in case of silent mode

	if (!dde_status.silent_mode) {

		targ   = -1;      
		prognum= -1;
		for( new p_idx=0; p_idx<sizeof( dde_setup.prog_start); p_idx++) {		
			new p_target = dde_setup.prog_target[p_idx];
			new p_start = dde_setup.prog_start[p_idx];
			new p_end   = dde_setup.prog_end[p_idx];
			new now   = now();
		  
			if (!p_start || !p_end || p_target < 0) continue;	// prog is disabled -> skip!
		  
			if (now > p_end) continue;	// prog is in the past -> skip!
	  
			new tx= -1;					// -1 = prog has no influence on effective target
			// prog is currently active 
			// -> take it's target "as is"
			if (now >= p_start) {
				tx= p_target;
			}	
			// prog is in the future 
			// -> interpolate ramp - e.g. calc min. required temp. to ramp up just in time (assumes 1.5°C/hr)
			// -> don't do this interpolation in OFF or PUMP-only mode (tx==0 or <=5)
			else if (now < p_start && p_target > 5) {
				tx= (p_target*3600*2 - (p_start-now) * 3) / 3600 / 2;	// 1.5°C -> 3/2				
				if (tx<=5) tx= -1;
			} 

			// remember the highest target temp found (and it's associated prog)
			if (tx > targ) {
				targ   = tx;
				prognum= p_idx+1;
			}			
		}	
		
		if (prognum==-1) targ= FALLBACK_TARGET;   // default if no fitting prog found
	}
	
	dde_status_set( 1, dde_status.prognum, prognum, "prognum");
	dde_status_set( 2, dde_status.target, targ, "target");

	return targ;
}

/**
 * append record to trend series
 */
dde_trend_append( stamp, Exx, tw_acc, tw_cnt, ticks, air, wp, htr, ubat, uusb, tsys){
	
    static t_prev_= 0;
    new tnow= now();

    // record only if we have a full period
    if (t_prev_) {

		new r[DDE_trend];

		r.total_tm  = tnow - tboot_;
		r.period_sec= tnow-t_prev_;

		r.Exx  = Exx;
		r.tair= -127;

		new cnt= max(1,tw_cnt);  // max(1,..) just for security
		r.tw= tw_acc / cnt;

		// convert from ticks (approx 500ms) into seconds
		new tks= max(1,ticks);  // avoid div 0 situations
		r.blower_sec= r.period_sec*air / tks;
		r.pump_sec  = r.period_sec*wp  / tks;
		r.heater_sec= r.period_sec*htr / tks;

		printf("TREND REC\n");
		printf("    blower %d = %d * %d / %d\n", r.blower_sec, r.period_sec, air, tks);
		printf("    pump   %d = %d * %d / %d\n", r.pump_sec,   r.period_sec, wp , tks);
		printf("    heater %d = %d * %d / %d\n", r.heater_sec, r.period_sec, htr, tks);

		// Ws
		new energy_air= dde_profile.blower_load*r.blower_sec;
		new energy_wp = dde_profile.pump_load*r.pump_sec;
		new energy_htr= dde_profile.heater_load*r.heater_sec;
		r.energy= energy_air + energy_wp + energy_htr;

		r.ubat= ubat;
		r.uusb= uusb;
		r.tsys= tsys;
		DDE_trend_write(stamp,r);
	}
    t_prev_= tnow;	
}