// === STATUS - up via config0 ===


// #define DDE_status_id (DDE_CONFIG+0) -> dde-core
// #define DDE_status_sz 31  -> dde-core
#define DDE_status [
	.blower_on,	// u8
	.heater_on,	// u8
	.pump_on,	// u8
	.Exx,	// u8
	.tair,	// s8
	.tw,	// s8
	.target,	// s8
	.prognum,	// s8
	.load,	// u16
	.silent_mode,	// u8
	.blower_mode,	// u8
	.stats_Exx,	// u8
	.stats_reset,	// stamp32
	.stats_energy,	// u16
	.stats_blower,	// u16
	.stats_heater,	// u16
	.stats_pump,	// u16
	.stats_twmin,	// s8
	.stats_twmax,	// s8
	.blower_trg,	// u8
	.silent_trg,	// u8
	.resetstats_trg,	// u8
	.live_trg,	// u8
]
stock DDE_status_write( const o[DDE_status]){
	dde_wbegin( DDE_status_id);
	dde_wu8     ( o.blower_on);
	dde_wu8     ( o.heater_on);
	dde_wu8     ( o.pump_on);
	dde_wu8     ( o.Exx);
	dde_ws8     ( o.tair);
	dde_ws8     ( o.tw);
	dde_ws8     ( o.target);
	dde_ws8     ( o.prognum);
	dde_wu16    ( o.load);
	dde_wu8     ( o.silent_mode);
	dde_wu8     ( o.blower_mode);
	dde_wu8     ( o.stats_Exx);
	dde_wstamp32( o.stats_reset);
	dde_wu16    ( o.stats_energy);
	dde_wu16    ( o.stats_blower);
	dde_wu16    ( o.stats_heater);
	dde_wu16    ( o.stats_pump);
	dde_ws8     ( o.stats_twmin);
	dde_ws8     ( o.stats_twmax);
	dde_wu8     ( o.blower_trg);
	dde_wu8     ( o.silent_trg);
	dde_wu8     ( o.resetstats_trg);
	dde_wu8     ( o.live_trg);
	dde_wend();
}
stock DDE_status_read( o[DDE_status]) {
	if (dde_rbegin( DDE_status_id)) return ERROR;
	dde_ru8     ( o.blower_on);
	dde_ru8     ( o.heater_on);
	dde_ru8     ( o.pump_on);
	dde_ru8     ( o.Exx);
	dde_rs8     ( o.tair);
	dde_rs8     ( o.tw);
	dde_rs8     ( o.target);
	dde_rs8     ( o.prognum);
	dde_ru16    ( o.load);
	dde_ru8     ( o.silent_mode);
	dde_ru8     ( o.blower_mode);
	dde_ru8     ( o.stats_Exx);
	dde_rstamp32( o.stats_reset);
	dde_ru16    ( o.stats_energy);
	dde_ru16    ( o.stats_blower);
	dde_ru16    ( o.stats_heater);
	dde_ru16    ( o.stats_pump);
	dde_rs8     ( o.stats_twmin);
	dde_rs8     ( o.stats_twmax);
	dde_ru8     ( o.blower_trg);
	dde_ru8     ( o.silent_trg);
	dde_ru8     ( o.resetstats_trg);
	dde_ru8     ( o.live_trg);
	dde_rend();
	return OK;
}

// === TREND - up via histdata0 ===


// #define DDE_trend_id (DDE_HISTDATA+0) -> dde-core
// #define DDE_trend_sz 25  -> dde-core
#define DDE_trend [
	.total_tm,	// s32
	.period_sec,	// u16
	.Exx,	// u8
	.tair,	// s8
	.tw,	// s8
	.energy,	// u32
	.blower_sec,	// u16
	.heater_sec,	// u16
	.pump_sec,	// u16
	.ubat,	// u16
	.uusb,	// u16
	.tsys	// s8
]
stock DDE_trend_write( const stamp=0, const o[DDE_trend]){
	dde_wbegin( DDE_trend_id);
	dde_ws32    ( o.total_tm);
	dde_wu16    ( o.period_sec);
	dde_wu8     ( o.Exx);
	dde_ws8     ( o.tair);
	dde_ws8     ( o.tw);
	dde_wu32    ( o.energy);
	dde_wu16    ( o.blower_sec);
	dde_wu16    ( o.heater_sec);
	dde_wu16    ( o.pump_sec);
	dde_wu16    ( o.ubat);
	dde_wu16    ( o.uusb);
	dde_ws8     ( o.tsys);
	dde_wend( stamp);
}

// === EVENTS - up via histdata1 ===


// #define DDE_events_id (DDE_HISTDATA+1) -> dde-core
// #define DDE_events_sz 63  -> dde-core
#define DDE_events [
	.code{12},	// astr
	.params{50},	// astr
]
stock DDE_events_write( const stamp=0, const code{12}, const params{50}){
	dde_wbegin( DDE_events_id);
	dde_wastr   ( code, 12);
	dde_wastr   ( params, 50);
	dde_wend( stamp);
}

// === CMDS - down via config5 ===


// #define DDE_cmds_id (DDE_CONFIG+5) -> dde-core
// #define DDE_cmds_sz 4  -> dde-core
#define DDE_cmds [
	.blower_trg,	// u8
	.silent_trg,	// u8
	.resetstats_trg,	// u8
	.live_trg,	// u8
]
stock DDE_cmds_read( o[DDE_cmds]) {
	if (dde_rbegin( DDE_cmds_id)) return ERROR;
	dde_ru8     ( o.blower_trg);
	dde_ru8     ( o.silent_trg);
	dde_ru8     ( o.resetstats_trg);
	dde_ru8     ( o.live_trg);
	dde_rend();
	return OK;
}

// === SETUP - down via config6 ===


// #define DDE_setup_id (DDE_CONFIG+6) -> dde-core
// #define DDE_setup_sz 94  -> dde-core
#define DDE_setup [
	.light,	// s32
	.prog_start[10],	// stamp32
	.prog_end[10],	// stamp32
	.prog_target[10]	// s8
]
stock DDE_setup_read( o[DDE_setup]) {
	if (dde_rbegin( DDE_setup_id)) return ERROR;
	dde_rs32    ( o.light);
	for(new j=0;j<10;j++) dde_rstamp32( o.prog_start[j]);
	for(new j=0;j<10;j++) dde_rstamp32( o.prog_end[j]);
	for(new j=0;j<10;j++) dde_rs8     ( o.prog_target[j]);
	dde_rend();
	return OK;
}

// === PROFILE - down via config8 ===


// #define DDE_profile_id (DDE_CONFIG+8) -> dde-core
// #define DDE_profile_sz 12  -> dde-core
#define DDE_profile [
	Float:.price_per_kwh,	// f32
	.blower_load,	// u16
	.heater_load,	// u16
	.pump_load,	// u16
	.debug_flags	// u16
]
stock DDE_profile_read( o[DDE_profile]) {
	if (dde_rbegin( DDE_profile_id)) return ERROR;
	dde_rf32    ( o.price_per_kwh);
	dde_ru16    ( o.blower_load);
	dde_ru16    ( o.heater_load);
	dde_ru16    ( o.pump_load);
	dde_ru16    ( o.debug_flags);
	dde_rend();
	return OK;
}
