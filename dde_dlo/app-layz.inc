/*************************************************************************************************
	LAYZ INTERFACE STACK

	new layz_bridge, layz_framedump
	layzInit( const port, const txbof, const rxbof, const onpacket{20}, const txto_cb{20}, const txto_ms)
	layzDump( const caption{}, port)
	layzSend( port, f2, f3, f4, const dump_caption{}="")
	
*************************************************************************************************/
const LAYZ_BOF_TX_DSP= 0xbb;	// BOF sent by DSP (rx by IOC)
const LAYZ_BOF_TX_IOC= 0xbc;	// BOF sent by IOC (rx by DSP)
const LAYZ_FRAME_SZ = 7;
const LAYZ_Exx_ITF_TIMEOUT = 0xff;	// Exx code used internally by itf driver to signal long lasting itf breakdowns

new layz_bridge=0;
new layz_framedump=0;

/*
	.cnt_cto - rx process canceled due to character timeout
	.cnt_edat[7] - // data error at frame position x
*/
#define Tlayz[ 
	.port, .oncto{20}, 											// pre-sets (internal)
	.nick{10}, .txbof, .rxbof, .onpacket{20}, .txto_cb{20}, .txto_ms, 		// settings configrd upon init
	.rxdiff{LAYZ_FRAME_SZ}, .txdiff{LAYZ_FRAME_SZ},				// buffers for diff detection + logging
	.rxbuf{LAYZ_FRAME_SZ}, .rxlen, 								// rx buffer 
	.cnt_txframes, .cnt_rxframes, .cnt_cto, .cnt_edat[7], .cnt_dubious	// stats/health output
]
	
new layz0[Tlayz] = [0,"_layz0_on_cto"];
new layz1[Tlayz] = [1,"_layz1_on_cto"];

layz( z[Tlayz], port){
  z= port ? layz1 : layz0;
}

_layzResetHealth( z[Tlayz]){
	// z.cnt_txframes 
	// z.cnt_rxframes
	z.cnt_cto= 0;
	z.cnt_dubious= 0;
	for( new j=0;j<sizeof(z.cnt_edat);j++) z.cnt_edat[j]= 0;
}
layzCheckHealth( port){
	
	new z[Tlayz];
	layz( z, port);
	
	// do NOT consider z.cnt_txframes || z.cnt_rxframes 
	new wanna_log= z.cnt_cto || z.cnt_dubious;
	for( new j=0;j<sizeof(z.cnt_edat);j++) wanna_log = wanna_log || z.cnt_edat[j];
	
	if (wanna_log) {
		#define b z.cnt_edat
		new s{50};
		sprintf( s,_,"%s tx=%d rx=%d dub=%d cto=%d edat=%d %d %d %d %d %d %d",
			z.nick, 
			z.cnt_txframes,
			z.cnt_rxframes,		
			z.cnt_dubious,
			z.cnt_cto, 
			b[0], b[1], b[2], b[3], b[4], b[5], b[6]);
		dde_event( "health-layz", s);
		#undef b
	}
	
	_layzResetHealth( port ? layz1 : layz0);
}


const {
	LAYZ_DUMP_RX= 0,
	LAYZ_DUMP_TX= 1
}


_layzHasDiff( diff{LAYZ_FRAME_SZ}, const frame{LAYZ_FRAME_SZ}) {
	new hasdiff=0;
	for( new i=0; i<LAYZ_FRAME_SZ; i++) {
		if (diff{i} != frame{i}) hasdiff= 1;
		diff{i} = frame{i};
	}
	return hasdiff;
}
layzDumpFrame( port, txrx, const frame{LAYZ_FRAME_SZ}) {
	
	if (!layz_framedump) return;
	
	new hasdiff;
	if (port) {
		hasdiff= txrx
			? _layzHasDiff( layz1.txdiff, frame)
			: _layzHasDiff( layz1.rxdiff, frame);
	}
	else {
		hasdiff= txrx
			? _layzHasDiff( layz0.txdiff, frame)
			: _layzHasDiff( layz0.rxdiff, frame);
	}

	if (hasdiff) {
		new s{50};
		sprintf( s, _, "|%d| %02x %02x %02x %02x %02x %02x %02x\n",
			port, 
			frame{0}, frame{1}, frame{2}, frame{3}, frame{4}, frame{5}, frame{6});	
			
		new code{12};			
		// --- workaround for pawn bug not correctly recognising {} inside named arrays ---
		for( new i=0;i<10;i++) code{i}= port ? layz1.nick{i} : layz0.nick{i};
		strcat( code, txrx==LAYZ_DUMP_TX ? "-tx":"-rx");
		dde_event( code, s);
	}
}


// buf : Uint8Array[7]
// returns: number 0..255
_layz_calc_cs( const frame{}){
  return 0xff & (frame{1} + frame{2} + frame{3} + frame{4});
}

_layz_setup( layz[Tlayz], const nick{10}, const txbof, const rxbof, const onpacket{20}, const txto_cb{20}, const txto_ms){
	layz.nick= nick;
	layz.txbof= txbof;
	layz.rxbof= rxbof;
	layz.onpacket= onpacket;
	layz.txto_cb= txto_cb;
	layz.txto_ms= txto_ms;
	layz.rxlen= 0;
	layz.cnt_txframes= 0;
	layz.cnt_rxframes= 0;
	layz.cnt_dubious= 0;
	layz.cnt_cto= 0;
	for( new i=0;i<7;i++) layz.cnt_edat[i]=0;
}
layzInit( const port, const nick{10}, const txbof, const rxbof, const onpacket{20}, const txto_cb{20}, const txto_ms) {

  static nthtime_=0;
  if (nthtime_) {
	nthtime_= 1;  
	setInterval( "layzMonitorTask", 60000);
  }	

  _layz_setup( port ? layz1 : layz0, nick, txbof, rxbof, onpacket, txto_cb, txto_ms);

  rM2M_UartClose(port);

  new fcb= funcidx( port ? "_layz1_on_rx" : "_layz0_on_rx");
  rM2M_UartInit(
    port,
    9600,
    RM2M_UART_8_DATABIT | RM2M_UART_PARITY_NONE | RM2M_UART_1_STOPBIT,
    fcb);
}

_layz_on_rxchunk( layz[Tlayz], const chunk{}, len){

	clearTimeout( layz.oncto);
	
	new err= 0;	
	for( new i=0; i<len; i++) {
		new c= chunk{i};
		layz.rxbuf{ layz.rxlen++}= c;
	
		switch (layz.rxlen) {
			case 1: err= c!=layz.rxbof;
			case 2: err= c!=0x03;
			case 3,4,5: err= 0;			
			case 6: err= c!=_layz_calc_cs(layz.rxbuf);
			case 7: {
				err= c!=0xfd;
				if (!err) {
					layz.cnt_rxframes++;
					setTimeout( layz.onpacket, 0);
					layz.rxlen= 0;
					
					clearTimeout( layz.txto_cb);	// clear any pending tx timeout					
				}
			}
		}
		
		if (err) {
			layz.cnt_edat[layz.rxlen-1]++;  // note data error		
			layz.rxlen= 0;		
		}		
	}	
	
	// continue receiving of frame (or wait for BOF of next frame)
	if (layz.rxlen) setTimeout( layz.oncto, 50);
}

forward _layz0_on_cto();
public _layz0_on_cto(){
  layz0.cnt_cto++;
  layz0.rxlen= 0;
}
forward _layz1_on_cto();
public _layz1_on_cto(){
  layz1.cnt_cto++;
  layz1.rxlen= 0;
}

forward _layz0_on_rx( const chunk{}, len);
public _layz0_on_rx( const chunk{}, len){
  if (layz_bridge) rM2M_UartWrite( 1, chunk, len);
  _layz_on_rxchunk( layz0, chunk, len);
}
forward _layz1_on_rx( const chunk{}, len);
public _layz1_on_rx( const chunk{}, len){
  if (layz_bridge) rM2M_UartWrite( 0, chunk, len);
  _layz_on_rxchunk( layz1, chunk, len);
}


_layz_inc_cnt_txframes( layz[Tlayz]){
	layz.cnt_txframes++;
}

layzSend( port, f2, f3, f4){
  if (layz_bridge) {
	dde_event( "ifault", "layzSend not available due to bridge mode active");
	return;
  }
  new z[Tlayz];
  layz( z, port);
  new txframe{LAYZ_FRAME_SZ};
  txframe{0}= z.txbof;
  txframe{1}= 0x03;
  txframe{2}= f2;
  txframe{3}= f3;
  txframe{4}= f4;
  txframe{5}= _layz_calc_cs( txframe);
  txframe{6}= 0xfd;

  _layz_inc_cnt_txframes( port ? layz1: layz0);
  
  rM2M_UartWrite( port, txframe, LAYZ_FRAME_SZ);
  layzDumpFrame( port, LAYZ_DUMP_TX, txframe);
  
  // setup reply timeout watcher
  setTimeout( z.txto_cb, z.txto_ms);
} 

