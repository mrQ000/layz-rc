/*

todo

	ausschlachten? -> C:\Users\aai\Documents\__BIG ONES\_projects\MT blinds\rmframewrk.p

	_upl_init_needs &= ~_UPL_NEED_TIME;  // timebase is guaranteed to be sync'd as soon as any config received
		// todo this methology may fail if there are no configs for download
		// e.g. use timer-loop simply checking the timebase validity if there are no mandatory configs to wait for!

*/


// === to be declared by application ===
// uplink not ready yet - waiting for configs from backend - switch app into PROG operation
// this callback is not called only during boot, but may be skipped if all DN configs are available from flash
forward onUplinkWaiting();
//  uplink ready, downward configs available - switch app into NORMAL operation
forward onUplinkReady();
// configX received from backend (or loaded from flash during boot)
forward onUplinkReceived( const cx);


const{
  // rmOpen MSIM communication mode (choose one)
  UPLINK_TRIG   = RM2M_TXMODE_TRIG,    // triggered/interval mode - connect with server up on rmFlush and disconnect afterwards
  UPLINK_WAKEUP = RM2M_TXMODE_WAKEUP,  // wakeup - same as above, but allow server to wake me up anytime
  UPLINK_ONLINE = RM2M_TXMODE_ONLINE,  // online - connect immediately with server and stay online;

  UPLINK_NOPOS = 0x1000, 			   // flag - do NOT fetch position information up on each uplinkFlush
};



new _upl_mode = -1;	// not initialized yet
new _upl_txstart_flags;

// current uplink initialisation state
// set during init-phase if some (mandatory) downward cx is not
// available on the device - e.g. needs to be loaded upon next connection with backend
#define  _UPL_NEED_C0 	0x0001  // 0x0200 = C9
#define  _UPL_NEED_TIME 0x8000
new _upl_init_needs;   			// 0=need nothing/ready, otherwiese _UPL_NEED_xxx bitmask




/* handle config updates received from backend
 */
forward _onUplinkConfig(const cx);
public _onUplinkConfig(const cx){

  if (!_DDE_CONFIGS_down[cx]) return; // ignore any change of none-downward config!

  new is_init= !!_upl_init_needs;

  _upl_init_needs &= ~_UPL_NEED_TIME;  // timebase is guaranteed to be sync'd as soon as any config received
  // todo this methology may fail if there are no configs for download
  // e.g. use timer-loop simply checking the timebase validity if there are no mandatory configs to wait for!

  if (onUplinkReceived( DDE_CONFIG+cx) >= 0) _upl_init_needs &= ~(_UPL_NEED_C0 << cx);
  if (is_init && !_upl_init_needs) onUplinkReady();  // ...when all mandatory configs loaded
}


new _upl_init_step=-1;
forward _on_upl_init();
public _on_upl_init(){

	++_upl_init_step;	// process next step

	// try to restore DOWNward configs
	if (_upl_init_step < 10){
		new cx = _upl_init_step;

		if (_DDE_CONFIGS_down[cx]){
			new err= onUplinkReceived( DDE_CONFIG+cx);
			// could not load mandatory config -> wait for uplink connection
			if (err<0) _upl_init_needs |= (_UPL_NEED_C0 << cx);
		}
		setTimeout( "_on_upl_init", 100);
	}
	// deferred restore of (mandatory) DOWNward configs
	// upon next backend connection
	else if (_upl_init_needs) {
		// connect immediately with BE and wait for timebase + mandatory configs
		uplinkMode( UPLINK_ONLINE);
		onUplinkWaiting();	// signal "not ready yet - waiting for configs from backend"
		// ... onUplinkConfig triggers onUplinkReady() as soon as everything's fine
	}
	// everything's fine - handover control to app
	else {
		onUplinkReady()
	}

	// stop init sequence! - no more setTimeout( "_on_upl_init", ...
}



/* init rapidM2M uplink
 * -----------------------------------------------------------------------------------------------------
 * prepare uplink for regular operation
 * 		- starts (and holds!) ONLINE mode if downward configs or timebase are missing
 *		- ***app must set uplinkMode*** upon onUplinkReady()
 * note: use uplinkFlush to issue immediate transmissions, especially in UPLINK_TRIG and UPLINK_WAKEUP mode
 */
stock initUplink(){

	assert( _upl_mode == -1, "initUplink");	// allow init just once

	// todo #log( "initUplink buffer[%d]", _DDE_BUF_BYTES);
	printf( "initUplink buffer /w %d bytes\n", _DDE_BUF_BYTES);
	errchk( rM2M_CfgOnChg( _funcidx( "_onUplinkConfig")));

	// try to init timebase & configs, or set flag to signal deferred init after first uplink connect
	_upl_init_needs= 0;

	// if timebase NOT valid -> wait for uplink connection to sync timebase
	// in the meantime do !!not write back anything!!
	if (rM2M_GetTime() < 3600*24*365*15) {
		_upl_init_needs |= _UPL_NEED_TIME;
	}

	// do further init in several micro steps to keep runtime's timeslice stable
	_upl_init_step= -1;
	_on_upl_init();
}



/* trigger immediate transmission
 * -----------------------------------------------------------------------------------------------------
 * requires initUplink() before
 */
stock flushUplink(){

	assert( _upl_mode != -1, "uplinkFlush");
	errchk( rM2M_TxStart( _upl_txstart_flags));
}

/* set/get uplink mode
 * -----------------------------------------------------------------------------------------------------
 * upl_mode: optional; when given, changes uplink mode (only if mode is not already set)
 * result: current uplink mode UPLINK_xxx (w/o UPLINK_NOPOS flag)
 */
stock uplinkMode( const upl_mode=-1){
	if (upl_mode != -1){

	  new fl = (upl_mode & UPLINK_NOPOS) ? 0 : 1;
	  new mo =  upl_mode & ~UPLINK_NOPOS;

	  if (_upl_mode != mo || _upl_txstart_flags != fl) {		// set it only if really changed...
		errchk( rM2M_TxSetMode( mo));
		_upl_mode	       =  mo;
		_upl_txstart_flags = fl
		if (UPLINK_ONLINE == mo) flushUplink();
	  }
	}
	return _upl_mode;
}
