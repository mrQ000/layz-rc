
#include <string>


 /* ===
	LOG
 === */
 
#define COUT_CHUNK_DELAY 	40  	// IOT-box: min. 35
#define COUT_CHUNK_CHARS 	120 	// IOT-box: max. 128
#define COUT_BUF_CHARS 		2*2048  // must be 2^n 
new _cout_buf{COUT_BUF_CHARS};
new _cout_r,_cout_w;

#define LOG_TMP_CHARS   200					// should be multiple of 4
#assert LOG_TMP_CHARS >= COUT_CHUNK_CHARS	// uses the same buffer
new _log_tmp{LOG_TMP_CHARS};	// used by log() *and* _coutTask
new _log_startstamp; // up on init: = rM2M_GetTime();


// flush next line from console output buffer
// called directly from 100ms timer source for efficiency
forward _coutTask();
public _coutTask(){ 

	if (_cout_r == _cout_w) {
		setTimeout("_coutTask", 100);
		return;	// nothing to output
	}
	
	new i=0;	
	while( (i < (COUT_CHUNK_CHARS-1)) && (_cout_r != _cout_w)){

		_log_tmp{i}= _cout_buf{_cout_r};
		_cout_r = ++_cout_r & (COUT_BUF_CHARS-1);
		i++;
	}	
	_log_tmp{i}= 0;
	print( _log_tmp);
	setTimeout("_coutTask", COUT_CHUNK_DELAY);
}

stock _cout(const s{}){

  for( new i=0; s{i}; i++){
    _cout_buf{_cout_w}= s{i};	
    _cout_w = ++_cout_w & (COUT_BUF_CHARS-1);
    if( _cout_w==_cout_r) {
      printf("\n\n*** ERROR console/log output - buffer overflow %d/%d ***\n\n",_cout_w,_cout_r);    
    }
  }
}

// dummy: just to eliminate compiler warning when called from "log((" which hands over result (number of chars) from sprintf
stock __log( dummy /*, _log_tmp as global/implicit param*/){	
	// todo logmsg starts with * -> force reset, ! -> error
	new s{10};
	sprintf( s,_,"%06d ", rM2M_GetTime()-_log_startstamp);
	_cout( s);
	_cout( _log_tmp);
	_cout( "\n");
	
	if ('*' == _log_tmp{0} && '*' != _log_tmp{1}) {
		print( "**********************\n>>> SYSERR > REBOOTING...\n\n\n");
		new i=1;
		while( i) i++;   // halt any processing - runtime will fire reboot
	} 	
#pragma warning push 
#pragma warning disable 203	// just to eliminate "dummy: symbol never used" warning
}
#pragma warning pop

/* log(( msg,p1,p2,...));
 *
 * msg: string with format instructions (similar to printf)
 *		first char: 
 *			'*' > system error, fire reboot
 * p1,
 * p2,
 * ...: optional parameters (similar to printf)
 */		
 // todo restore
#define _____log((  __log( sprintf(_log_tmp,_,


/* replacement for assert() - workaround 'cause assert seems to be declared, but has no effect */
#pragma warning push 
#pragma warning disable 201
#define assert(%1,%2) __assert(%1,__line,%2)
#pragma warning pop

stock __assert( const isOk, const line, const msg{}="-"){

  if (!msg{0}) __assert( false, line, "__assert() missing msg!");
  if (!isOk) {
    printf( "\r\n\r\n* ASSERTION @%d %s\r\n", line, msg);
    print( "**********************\n\r>>> REBOOTING...\n\r\n\r\n\r");
    new i=1;
    while( i) i++;   // halt any processing - runtime will fire reboot
  }
}


const {
  ERROR_FULL           = -21,  // tx buffer full (e.g.uart)
  ERROR_FEATURE_LOCKED = -103, // e.g. easyV3 RS232, RS485 need separate unlock
  ERROR_NMEA_DATATYPE  = RM2M_NMEA_ERR_DATATYPE  - 700, // -2, 		
  ERROR_NMEA_SENTENCE  = RM2M_NMEA_ERR_SENTENCE  - 700, // -3, 
  ERROR_NMEA_LATITUDE  = RM2M_NMEA_ERR_LATITUDE  - 700, // -4, 
  ERROR_NMEA_LONGITUDE = RM2M_NMEA_ERR_LONGITUDE - 700, // -5, 
  ERROR_NMEA_ALTITUDE  = RM2M_NMEA_ERR_ALTITUDE  - 700, // -6, 
  ERROR_NMEA_SATCNT    = RM2M_NMEA_ERR_SAT_USED  - 700, // -7, 
  ERROR_NMEA_QUALITY   = RM2M_NMEA_ERR_QUAL      - 700, // -8, 
  ERROR_NMEA_FORMAT    = -750,
  ERROR_UNKOWN_PORT	   = -900,
  ERROR_TIMEOUT		   = -901,
  ERROR_FRAME		   = -902
};

_errToStr( const errcode){

  new s{50};							// *=sod !=error -=info others=warning
  switch( errcode){
    case ERROR                    : s="*E_ERROR";	 		   // e.g. invalid interface number, uart buffer full...
    case ERROR_PARAM              : s="*E_PARAM";               // Parameter error
    case ERROR_UNKNOWN_HDL        : s="*E_UNKNOWN_HDL";         // Unknown handler, handle or resource error
    case ERROR_ALREADY_SUBSCRIBED : s="*E_ALREADY_SUBSCRIBED";  // Already subscribed service or resource error
    case ERROR_NOT_SUBSCRIBED     : s="*E_NOT_SUBSCRIBED";      // Not subscribed service error
    case ERROR_FATAL              : s="*E_FATAL";               // Fatal error
    case ERROR_BAD_HDL            : s="*E_BAD_HDL";             // Bad handle or resource error
    case ERROR_BAD_STATE          : s="*E_BAD_STATE";           // Bad state error
    case ERROR_PIN_KO             : s="*E_PIN_KO";              // Bad PIN state error
    case ERROR_NO_MORE_HANDLES    : s="*E_NO_MORE_HANDLES";    // The service subscription maximum capacity is reached
    case ERROR_DONE               : s="*E_DONE";               // The required iterative process is now terminated
    case ERROR_OVERFLOW           : s="*E_OVERFLOW";           // The required operation has exceeded the function capabilities
    case ERROR_NOT_SUPPORTED      : s="*E_NOT_SUPPORTED";      // An option, required by the function, is not enabled on the CPU, the function is not supported in this configuration
    case ERROR_NO_MORE_TIMERS     : s="*E_NO_MORE_TIMERS";     // The function requires a timer subscription, but no more timer resources are available
    case ERROR_NO_MORE_SEMAPHORES : s="*E_NO_MORE_SEMAPHORES"; // The function requires a semaphore allocation, but there are no more semaphores resources
    case ERROR_SERVICE_LOCKED     : s="*E_SERVICE_LOCKED";     // The function was called from a low or high level interrupt handler (the function is forbidden in this case)
	case ERROR_FEATURE_LOCKED     : s="*E_FEATURE_LOCKED";

	case ERROR_FULL               : s="!E_FULL"; 				// buffer full (e.g. uart)
	
    case ERROR_NMEA_DATATYPE      : s="!E_NMEA_DATATYPE"; 	// Datatype (e.g. $GGSA) is not supported
    case ERROR_NMEA_SENTENCE      : s="!E_NMEA_SENTENCE";	// Sentence is not valid (e.g. checksum error)
    case ERROR_NMEA_LATITUDE      : s="!E_NMEA_LATITUDE"; 	// Latitude value is not valid
    case ERROR_NMEA_LONGITUDE     : s="!E_NMEA_LONGITUDE"; 	// Longitude value is not valid
    case ERROR_NMEA_ALTITUDE      : s="!E_NMEA_ALTITUDE"; 	// Altitude value is not valid
    case ERROR_NMEA_SATCNT        : s="!E_NMEA_SATCNT";  	// Number of satellites used is not valid
    case ERROR_NMEA_QUALITY       : s="!E_NMEA_QUALITY"; 	// quality entry is not supported
	case ERROR_NMEA_FORMAT		  : s="!E_NMEA_FORMAT";		// the NMEA frame contains valid, but unexpected data/characters
	
	case ERROR_UNKOWN_PORT 		  : s="*E_UNKOWN_PORT";		// HAL does not support given port
	case ERROR_TIMEOUT			  : s="!E_TIMEOUT";			
	case ERROR_FRAME		      : s="!E_FRAME";			// faulty message frame (crc error, invalid content,..)
	
	case ERROR_MEM 				  : s="*E_MEM";	//	= -100,// error allocating memory
	case ERROR_SIM_STATE 		  : s="!E_SIM_STATE";	// = -101,// SIM state error
    default                       : sprintf( s, _, "*UNKOWN ERROR #%d", errcode);
  }
  return s;
}

/*
	xe, xs: 
	-, -    none given    - report any error using default messages
	N, -    xe only given - ignore given error code
	N, ".." both given    - use this message xs for error reporting (instead of default msg)
*/
stock errchk( const e, const xe=0, const xs{}=""){
  if (e < 0) {
    if (e == xe) {
	  if (xs{0}) { print(xs); print("\n"); }
	  // todo restore log(( xs));
	}  
    // todo restore else log(( _errToStr(e)));
    else { print(_errToStr(e)); print("\n"); }
  }
  return e;
}



stock memcpy( dst{},const dstofs,const src{},const srcofs,const bytes,const dst_cells=sizeof dst, const src_cells=sizeof src){
	
  assert( dstofs+bytes <= 4*dst_cells, "memcpy:dst out of range");
  assert( srcofs+bytes <= 4*src_cells, "memcpy:src out of range");

  if      (!srcofs) errchk( rM2M_SetPackedB( dst, dstofs, src, bytes));
  else if (!dstofs) errchk( rM2M_GetPackedB( src, srcofs, dst, bytes));
  else for( new i=0; i<bytes; i++) dst{dstofs+i}= src{srcofs+i};  
}

stock memcmp( const src1{}, const src1ofs, const src2{}, const src2ofs, bytes, src1cells=sizeof src1, src2cells=sizeof src2){
  assert( (src1ofs+bytes) <= 4*src1cells, "memcmp:src1 out of range");
  assert( (src2ofs+bytes) <= 4*src2cells, "memcmp:src2 out of range");
  for( new i=0; i<bytes; i++){
    if (src1{src1ofs+i} != src2{src2ofs+i}) return 1;
  }
  return 0;
}

// convert byte buffer into hex-string
stock memtostr( str{}, const mem{}, const len, const str_cells=sizeof str){

  assert( str_cells*4 >= len*3, "memtostr.1");

  new HEX_DIGITS{} = "0123456789abcdef";

  for( new i=0;i<len;i++) {
    str{i*3+0}= HEX_DIGITS{mem{i} >> 4};
    str{i*3+1}= HEX_DIGITS{mem{i} & 0x0f};
    str{i*3+2}= ' ';
  }
  if (len) str{len*3-1}= 0; // convert last space into #0
  else     str{0}= 0;
  
  return str;
}


stock _funcidx(const cb{}){

  new f = funcidx( cb);
  // todo restore if (-1 == f) log((  "*unkown callback <%s>", cb));  
  if (-1 == f) printf((  "*unkown callback <%s>\n", cb));  
  return f;
}




/* ===
	TIMER
	
	uses system-tick underneath:		IOT-box: 683 ticks/sec		M120: 18.5 ticks/sec
 === */

/* register timeout callback
 * with auto-clear if given callback has been already registered before
 */
stock setTimeout( const cb{}, const ms){
	
  new f= _clearTimeoutInterval( cb);
  // ms < 1 ? 1 : ms ... todo rem --- temporary bug fix 'cause runtime does not fire callback then ms<=0
  errchk( rM2M_TimerAddExt( f, false, ms < 1 ? 1 : ms), ERROR, "setTimeout: out of timers");	
}

/* register interval callback
 * with auto-clear if given cb has been already registered before
 * uses exact 1s timer if ms==1000 
 */
stock setInterval(const cb{}, const ms){
  new f = _clearTimeoutInterval( cb);	// clear timer and get funcidx of callback
  
  if (ms == 1000) errchk( rM2M_TimerAdd( f),              ERROR, "setInterval: out of timers");
  else            errchk( rM2M_TimerAddExt( f, true, ms), ERROR, "setInterval: out of timers"); 
}

#define clearInterval(%1) _clearTimeoutInterval(%1)
#define clearTimeout(%1) _clearTimeoutInterval(%1)

stock _clearTimeoutInterval( const cb{}){
  new f = _funcidx( cb);
  if (f<OK) return f;						// abort immediately in case of any error
  errchk( rM2M_TimerRemove( f), ERROR);
  errchk( rM2M_TimerRemoveExt( f), ERROR); 
  return f; 								// return cb handle -> used by setTimeout/setInterval internally only!
}


new _pin_dir{PIN_COUNT};

stock pin_input( pin){
  rM2M_GpioDir( pin, 0);  // output
  _pin_dir{pin}= 0;
}
stock pin_write( pin, setpin){
  if (!_pin_dir{pin}) {
    rM2M_GpioDir( pin, 1);  // output
    _pin_dir{pin}= 1;
  }
  rM2M_GpioSet( pin, setpin);
}
stock pin_read( pin){
  return rM2M_GpioGet( pin);
}