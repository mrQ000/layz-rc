/*************************************************************************************************
  LAYZ IO CONTROLLER EMULATOR
*************************************************************************************************/

// current status of ioc emulator (reported to phy-display)
new ioc_out[.temp,.booted,.err]= [0,0,0];

new ioc_txto_cnt_=0;

forward on_rx_ioc();
public on_rx_ioc(){

	if (ioc_txto_cnt_ >= 3) {	
		new s{50};
		sprintf( s,_,"back after %d fails",ioc_txto_cnt_);
		dde_event( "ioc-txto", s);
		ioc_txto_cnt_ = 0;
	}	

  new z[Tlayz];
  layz(z,IOC_EMU);
  layzDumpFrame( IOC_EMU, LAYZ_DUMP_RX, z.rxbuf);

  // validate payload of rxframe (others already checked by layz_rx_char)
  // (e.g. check if unused bits are all zero)
  new ok= ((z.rxbuf{2} & 0xc4) == 0x00) ||
          ((z.rxbuf{3}) == 0x00) ||
          ((z.rxbuf{4} & 0xfe) == 0x00);
  if (!ok) {
    IOC_layz.cnt_dubious++;
    return;
  }

  if (layz_bridge) return;

  // request from physical DSP came in - respond with current status

  layzSend( IOC_EMU,
    ioc_out.temp,
    ioc_out.err,
    ioc_out.booted?0x01:0x00);
}
forward on_txto_ioc();
public on_txto_ioc(){
	if (++ioc_txto_cnt_ == 3) {
		dde_event( "ioc-txto", "IOC tx timeout - physical display connected?");
	}	
}
