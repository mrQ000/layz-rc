/*************************************************************************************************
	BRIDGE CONTROL

	run_as_bridge()

*************************************************************************************************/

/*
	open UART port for layz stack communication

	port - UART port 0/1
	cb:function( const chunk{}, len) - rx callback
*/
layzOpen( const port, const cb{}) {

  rM2M_UartClose(port);

  new fcb= funcidx(cb);
  rM2M_UartInit(
    port,
    9600,
    RM2M_UART_8_DATABIT | RM2M_UART_PARITY_NONE | RM2M_UART_1_STOPBIT,
    fcb);
}





forward on_bridge_rxdsp( const chunk{}, len);
public on_bridge_rxdsp( const chunk{}, len){
  rM2M_UartWrite( COM_IOC, chunk, len);
}
forward on_bridge_rxioc( const chunk{}, len);
public on_bridge_rxioc( const chunk{}, len){
  rM2M_UartWrite( COM_DSP, chunk, len);
}

run_as_bridge(){
  layzOpen( COM_DSP, "on_bridge_rxdsp");
  layzOpen( COM_IOC, "on_bridge_rxioc");
}
