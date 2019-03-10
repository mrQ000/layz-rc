


#define DDE_CONFIG   0x20
#define DDE_HISTDATA 0x40

stock dde_init(){
	// does nothing at the moment
}

new _dde_buf{_DDE_BUF_BYTES};
new _dde_cid=0;
new _dde_idx;

// used to minimize data amount on writes when container is marked as "up+"
// counts bytes occupied without any trailing fill-bytes (such as unused chars in astr)
new _dde_idx_flex;

new _dde_size;
new _dde_cx_force_rec=0; // bit N = do not use change detection when recording configN

// cid = container id incl. DDE_CONFIG or DDE_HISTDATA
stock _dde_buf_lock( const cid){

  assert( !_dde_buf_islocked(), "_dde_buf_lock.1");

  new isize=cid & DDE_CONFIG
	? _DDE_CONFIGS_size[cid & ~DDE_CONFIG]
	: _DDE_HISTS_size[cid & ~DDE_HISTDATA];

  assert( isize <= _DDE_BUF_BYTES, "_dde_buf_lock.2");
  assert( isize > 0, "_dde_buf_lock.3");

  _dde_size= isize;
  _dde_idx = 0;
  _dde_cid = cid;
  return isize;
}

stock _dde_buf_unlock(){
  assert( _dde_buf_islocked(), "_dde_buf_unlock");  // check if buf is already locked
  _dde_cid= 0;
}

stock _dde_buf_islocked(){
	return _dde_cid != 0;
}


// READ methods

/* get value as integer number
*/
stock dde_ru8( &val, const def=0){
  if ((_dde_idx+1) <= _dde_size){
    val= _dde_buf{_dde_idx};
  }
  else val= def;
  _dde_idx += 1;
}

stock dde_rs8( &val, const def=0){
  if ((_dde_idx+1) <= _dde_size) {
    val = _dde_buf{_dde_idx};                 // alternative: errchk( rM2M_GetPacked( _dde_buf, _dde_idx, n, 1, true));
    if (val > 127) val -= 256;      // sign extension
  }
  else val = def;
  _dde_idx += 1;
}

stock dde_ru16( &val, const def=0){
  if ((_dde_idx+2) <= _dde_size){
    val= 0x100*_dde_buf{_dde_idx} + _dde_buf{_dde_idx+1};
  }
  else val = def;
  _dde_idx += 2;
}

stock dde_rs16( &val, const def=0){
  if ((_dde_idx+2) <= _dde_size) {
    errchk( rM2M_GetPacked( _dde_buf, _dde_idx, val, 2, true));
  }
  else val = def;
  _dde_idx += 2;
}

stock dde_rs32( &val, const def=0){
  if ((_dde_idx+4) <= _dde_size) {
    errchk( rM2M_GetPacked( _dde_buf, _dde_idx, val, 4, true));
  }
  else val = def;
  _dde_idx += 4;
}

// !!! quick & dirty for U32 values, but
// consider that any U32 with bit31=1 will result in a negative number !!!
#define dde_ru32 dde_rs32
#define dde_rstamp32 dde_rs32

/* get value as float f32 */
stock dde_rf32( &Float:val, const Float:def=0.0){
  if ((_dde_idx+4) <= _dde_size) {
    errchk( rM2M_GetPacked( _dde_buf, _dde_idx, val, 4, true));
  }
  else val = def;
  _dde_idx += 4;
}

/* get value(s) as binary
	consider that multi-byte numerics use BIG ENDIAN
  val: output buffer; bytes not supplied by uplink buffer are filled-up with <def> or 0
  def: -> val
  len: number of bytes to transfer
*/
stock dde_rbuf( val{}, const len, const def{}="\x00\x00\x00\x00", const val_cells=sizeof val, def_cells=sizeof def){

  assert( len       >  0,           "rmr_buf:len must be >0")
  assert( len       <= val_cells*4, "rmr_buf:len must be <val length");
  assert( def_cells <= val_cells  , "rmr_buf.def length must be <val length");

  new srclen= min( len, _dde_size - _dde_idx);  // avoid reading behind end of _dde_buf


  if (srclen <= 0) {     // preset with default values if src can not supply all bytes
	memcpy( val, 0, def, 0, min( len, def_cells*4), val_cells, def_cells);	// val= def;
    for( new j=4*def_cells; j<4*val_cells; j++) val{j}=0; // fill tail with 0
  }

  else {           // read from _dde_buf as much as possible
	memcpy( val, 0, _dde_buf, _dde_idx, srclen, val_cells, sizeof _dde_buf)
  }

  _dde_idx += len;
}
#define dde_rastr dde_rbuf

// WRITE methods

#define dde_wu8 dde_wsu8_
#define dde_ws8 dde_wsu8_
stock dde_wsu8_( const ival){

  assert( _dde_idx+1 <= _dde_size, "rmw_su8");
  _dde_buf{ _dde_idx}= ival;
  _dde_idx     += 1;
  _dde_idx_flex = _dde_idx;
}

#define dde_wu16 dde_wsu16_
#define dde_ws16 dde_wsu16_
stock dde_wsu16_( const ival){

  assert( _dde_idx+2 <= _dde_size, "rmw_su16");
  new i = ival;
  rM2M_SetPacked( _dde_buf, _dde_idx, i, 2, true);  //  mem{idx+1}  = val & 255; mem{idx}= (val >> 8) & 255;
  _dde_idx     += 2;
  _dde_idx_flex = _dde_idx;
}

#define dde_wu32 dde_wsu32_
#define dde_ws32 dde_wsu32_
#define dde_wstamp32 dde_wsu32_

stock dde_wsu32_( const ival){

  assert( _dde_idx+4 <= _dde_size, "rmw_su32");
  new i = ival;
  rM2M_SetPacked( _dde_buf, _dde_idx, i, 4, true);
  _dde_idx     += 4;
  _dde_idx_flex = _dde_idx;
}
stock dde_wf32( const Float:fval){
  assert( _dde_idx+4 <= _dde_size, "rmw_f32");
  new Float:f = fval;                       // workaround - SetPacked requires reference (not comp. with "const")
  rM2M_SetPacked( _dde_buf, _dde_idx, f, 4, true);
  _dde_idx     += 4;
  _dde_idx_flex = _dde_idx;
}
stock dde_wbuf( const src{}, const ilen){

  assert( _dde_idx+ilen <= _dde_size, "rmw_buf");
  rM2M_SetPackedB( _dde_buf, _dde_idx, src, ilen);
  _dde_idx     += ilen;
  _dde_idx_flex = _dde_idx;
}

stock dde_wastr( const src{}, const ilen_max){

  new ilen;
  for( ilen=0; ilen<ilen_max && src{ilen}; ) ilen++;	// safe version of strlen considering ilen_max
  if (ilen<ilen_max) ilen++;							// add terminating zero if string does not occupy the whole buffer

  assert( _dde_idx+ilen <= _dde_size, "dde_wastr");
  rM2M_SetPackedB( _dde_buf, _dde_idx, src, ilen);

  _dde_idx_flex = _dde_idx + ilen;
  _dde_idx     += ilen_max;
}

stock dde_wbegin( const cid){
  _dde_buf_lock( cid);
  // todo clr buf to 0 ?
  if (cid & DDE_HISTDATA) {
    _dde_buf{0} = cid & ~DDE_HISTDATA;   // put split-tag into front of histdata item
    _dde_idx++;
  }
}

/*
  stamp : for histdata only; optional; seconds since 31.12.1999 0:00
  result: 0=ok, <0 = error, >0 = stamp assigned to recording (/w histdata only)
*/
stock dde_wend( stamp=0){

	// todo rem
	// #log("dde_wend %c%d len=%d size=%d", (_dde_cid&DDE_CONFIG)?'C':'H', _dde_cid&~(DDE_CONFIG|DDE_HISTDATA), _dde_idx, _dde_size);	// todo rem

	assert( _dde_buf_islocked(), "dde_wend.1");			// dde_buf_lock required

	new e  = 0;
	new cid= _dde_cid;

	if (cid & DDE_CONFIG) {
		new cx= cid & ~DDE_CONFIG;
		assert( cx>=0 && cx <=9, "dde_wend.3");

		new bytes = _DDE_CONFIGS_varlen[cx] ? _dde_idx_flex : _dde_idx;

		// expect whole buffer to be set with new values (if container is not marked as flexible "up+")
		assert( _DDE_CONFIGS_varlen[cx] || bytes == _dde_size, "dde_wend.2c");

		new wanna_rec= _dde_cx_force_rec & (1<<cx);
		if (! wanna_rec){    	        // change detection
			new tmp{250+2};
			for( new si=0; si<_dde_size; si+=250){

				new tmpcnt= min( 250, _dde_size-si);
				e = rM2M_CfgRead( cid, si, tmp, tmpcnt);
				wanna_rec = wanna_rec || (e<=0);            // ERROR(-1) ... indicates that Cx does not exist yet
				wanna_rec = wanna_rec || (e != tmpcnt);     // stored length does not match current length?
				wanna_rec = wanna_rec || (memcmp( tmp, 0, _dde_buf, si, tmpcnt));	// modifications detected?
				if (wanna_rec) break;
			}
		}
		if (wanna_rec) e = errchk( rM2M_CfgWrite( cx, 0, _dde_buf, bytes));
	}
	else if (cid & DDE_HISTDATA){
		new hx= cid & ~DDE_HISTDATA;
		assert( hx>=0 && hx <=9, "dde_wend.4");

		new bytes= _DDE_HISTS_varlen[hx] ? _dde_idx_flex : _dde_idx;

		// expect whole buffer to be set with new values (if container is not marked as flexible "up+")
		assert( _DDE_HISTS_varlen[hx] || bytes == _dde_size, "dde_wend.2h");

		if (!stamp) stamp= rM2M_GetTime();    // if no app-stamp given, set stamp to "now"

		e= errchk( rM2M_RecData( stamp, _dde_buf, bytes),
			ERROR_PARAM,
			"!dde_wend.histdata buffer full");

	}
	else assert( false,"dde_wend.5");

	_dde_buf_unlock();

	return e<0 ? e : 0;
}


/*
  for Cx resources only!

???  - bei ändeurng länge: nur neue felder defaulten
		...rmr - verwendet default, wenn auf ein feld hinter gelesener size zugegriffen wird

version: optional; >0 check if loaded Cx has same version
    in case of version mismatch: report rlen=0 to reset whole Cx to defaults
result:
    <0 ... error, sets _dde_buf to "empty"
     0 ... OK
*/
stock dde_rbegin( const cid, const version=0){

  new cx= cid & ~DDE_CONFIG;
  assert((cx >= 0) && (cx <= 9), "dde_rbegin:cid must be 0..9");

  new size= _dde_buf_lock( cid);

  assert( size > 0, "dde_rbegin:size must be >0");

  new tolerant= !_DDE_CONFIGS_down[cx];		// be tolerant when NOT a DOWNward config

  // try to load Cx from uplink into tmp buffer
  new rlen = rM2M_CfgRead( cx, 0, _dde_buf, size);

  if (rlen == -1) rlen= 0;	// -1 = Cx does not exist yet - treat it the same ways as "Cx is empty"
  if (rlen < 0) {			// <0 = some error code
    _dde_size= 0;

    if (!tolerant) { 	
      _dde_buf_unlock();
      return errchk( rlen);
	}
  }

  // check if container has version info (u8 @ 0)
  // and version matches with current software
  if (version && rlen>0){
    new rver = _dde_buf{0};
    _dde_idx= 1;

    if (rver != version) {      // version mismatch? -> reset to defaults

      // todo #log( "C%d version mismatch len:%d>>%d ver:%d>>%d", cx, rlen, size, rver, version);
      // todo if (version < rver) #log( "  version DOWNGRADE warning - logic update required?"); // evt. besondere warnung wenn current version < downloaded version ist?
      printf( "C%d version mismatch len:%d>>%d ver:%d>>%d\n", cx, rlen, size, rver, version);
      if (version < rver) printf( "  version DOWNGRADE warning - logic update required?\n"); // evt. besondere warnung wenn current version < downloaded version ist?
      rlen= 0; // force default for all values incl. version
    }
  }

  // adapt cache size to effectively available bytes (and print warning)
  if (size != rlen) {
    // todo #log( "C%d size mismatch, expected %d but got %d bytes ... %s", cx, size, rlen, tolerant?"ignored":"error!");
	printf( "C%d size mismatch, expected %d but got %d bytes ... %s\n", cx, size, rlen, tolerant?"ignored":"error!");
	_dde_size= rlen;
	if (!tolerant) { 	// insist on exactly fitting config!
		_dde_buf_unlock();
		return ERROR;
	}
  }

  return OK;
}

stock dde_rend(){
  _dde_buf_unlock();
}


const {
	DDE_alert_SIGNAL_OFL = 0b00000001, // overflow, low pri ("warning")
	DDE_alert_SIGNAL_OFH = 0b00000010, // overflow, high pri ("alarm")
	DDE_alert_SIGNAL_UFL = 0b10000001, // underflow, low pri ("warning")
	DDE_alert_SIGNAL_UFH = 0b10000010, // underflow, high pri ("alarm")
	DDE_alert_SIGNAL_TFL = 0b00000100, // technical fault, low pri ("warning")
	DDE_alert_SIGNAL_TFH = 0b00001000  // technical fault, high pri ("alarm")
}
/* append new alert record
*		[in] signal     -> DDE_alarm_SIGNAL_xxx
*		[in] cid   		DDE_xxx_id - id of *histdata*-stream which raises the alert
*		[in] field    	DDE_xxx_fff_idx - index of field 0+ which raises the alert
*		[in] fValue     current value; float number or F32_SC/F32_UF/F32_OF/F32_OL
*						(DDE_alert_SIGNAL_TFx: set to F32_SC/F32_UF/F32_OF/F32_OL
*		[in] fThreshold threshold of alert
*						(DDE_alert_SIGNAL_TFx: use 0.0)
*/
stock DDE_alert_write( const signal, const cid, const field, const Float:fValue, const Float:fThreshold) {

  new hx= cid & ~DDE_HISTDATA;
  assert( hx>=0 && hx<=9, "DDE_alarm_write.1");	// no histdata container or out of h0..h9 range

  const { DDE_alert_sz = 13 };
  new x{DDE_alert_sz};
  x{0} = 0x20; // split tag
  x{1} = hx;
  rM2M_Pack(x,  2, field,    	RM2M_PACK_BE + RM2M_PACK_U16);
  x{4} = iTag;
  rM2M_Pack(x,  5, fValue,      RM2M_PACK_BE + RM2M_PACK_F32);
  rM2M_Pack(x,  9, fThreshold,  RM2M_PACK_BE + RM2M_PACK_F32);
  return rM2M_RecData(0, x, DDE_alert_sz);
}
stock DDE_alert_overflow( const hipri, const cid, const field, const Float:fValue, const Float:fThreshold) {
	return DDE_alert_write( hipri ? DDE_alert_SIGNAL_OFH : DDE_alert_SIGNAL_OFL, cid, field, fValue, fThreshold);
}
stock DDE_alert_underflow( const hipri, const cid, const field, const Float:fValue, const Float:fThreshold) {
	return DDE_alert_write( hipri ? DDE_alert_SIGNAL_UFH : DDE_alert_SIGNAL_UFL, cid, field, fValue, fThreshold);
}
stock DDE_alert_fault( const hipri, const cid, const field) {
	return DDE_alert_write( hipri ? DDE_alert_SIGNAL_TFH : DDE_alert_SIGNAL_TFL, cid, field, F32_OL, 0.0);
}
