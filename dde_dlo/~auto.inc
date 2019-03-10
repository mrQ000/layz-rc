// === PROJECT META ===

#define APM_ID            "seq34y"
#define APM_VERSION       1
#define APM_OWNER         "dev1@rapidm2m.studio"
#define APM_NAME          "LayZ-RC"
#define APM_AUTHOR        "dev1@rapidm2m.studio"
#define APM_REQUIRED_APP  ""
#define APM_REQUIRED_FW   ""
#define APM_REQUIRED_HW   "m3iotbox"
#define APM_HWID_m3iotbox 1
#define APM_HWID_M3IOTBOX 1
#define APM_REQUIRED_BE   ""
#define APM_CREATE_STAMP  "2018-11-28 15:55:57"
#define APM_CREATE_MS     1543420557974
#define APM_BUILD_STAMP   "2018-12-03 14:18:10"
#define APM_BUILD_MS      0

// === DDE CORE ===

#define _DDE_BUF_BYTES (94+10)

//                                c0	c1	c2	c3	c4	c5	c6	c7	c8	c9
stock _DDE_CONFIGS_size[]  =[ 31,	0,	0,	0,	0,	4,	94,	0,	12,	0];
stock _DDE_CONFIGS_up[]    =[ 1,	0,	0,	0,	0,	0,	0,	0,	0,	0]; // ..is tolerant when reading back container with under-length
stock _DDE_CONFIGS_down[]  =[ 0,	0,	0,	0,	0,	1,	1,	0,	1,	0]; // reading the container expects strict length
stock _DDE_CONFIGS_varlen[]=[ 0,	0,	0,	0,	0,	0,	0,	0,	0,	0]; // allow partial write (dde_wxxx)

//                               h0	h1	h2	h3	h4	h5	h6	h7	h8	h9
stock _DDE_HISTS_size[]   =[ 25,	63,	0,	0,	0,	0,	0,	0,	0,	0];
stock _DDE_HISTS_varlen[] =[ 0,	0,	0,	0,	0,	0,	0,	0,	0,	0]; // allow partial write (dde_wxxx)


// === STATUS - up via config0 ===

#define DDE_status_id (DDE_CONFIG+0)
#define DDE_status_sz 31

// === TREND - up via histdata0 ===

#define DDE_trend_id (DDE_HISTDATA+0)
#define DDE_trend_sz 25

// === EVENTS - up via histdata1 ===

#define DDE_events_id (DDE_HISTDATA+1)
#define DDE_events_sz 63

// === CMDS - down via config5 ===

#define DDE_cmds_id (DDE_CONFIG+5)
#define DDE_cmds_sz 4

// === SETUP - down via config6 ===

#define DDE_setup_id (DDE_CONFIG+6)
#define DDE_setup_sz 94

// === PROFILE - down via config8 ===

#define DDE_profile_id (DDE_CONFIG+8)
#define DDE_profile_sz 12
