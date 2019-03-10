/*************************************************************************************************
	BRIDGE CONTROL

	new layz_bridge
	layzInit( const port, const onpacket{20})
	layzDump( const caption{}, port)

*************************************************************************************************/
new layz_bridge= 0;

#define Tlayz[ .oncto{20}, .onpacket{20}, .rxbuf{10}, .rxlen, .ctocnt]
new layz0[Tlayz] = ["_layz0_on_cto"];
new layz1[Tlayz] = ["_layz1_on_cto"];

layz( z[Tlayz], port){
  z= port ? layz1 : layz0;
  return z;
}

layzDump( const caption{}, port){

  new z[Tlayz];
  layz( z, port);

  #define b z.rxbuf
  printf("%d:%s [%d] %02x %02x %02x %02x %02x %02x %02x\n",
    port, caption, z.rxlen,
    b{0}, b{1}, b{2}, b{3}, b{4}, b{5}, b{6}, b{7});
  #undef b
}

_layz_reset( layz[Tlayz], const onpacket{20}){
    layz.onpacket= onpacket;
    layz.rxlen= 0;
    layz.ctocnt= 0;
}
layzInit( const port, const onpacket{20}) {

  _layz_reset( port ? layz1 : layz0, onpacket);

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
  for( new i=0; i<len; i++) {
    layz.rxbuf{ layz.rxlen++}= chunk{i};
    if (layz.rxlen==7) {
      setTimeout( layz.onpacket, 0);
      layz.rxlen= 0;
    }
  }
  if (layz.rxlen) setTimeout( layz.oncto, 50);
}

forward _layz0_on_cto();
public _layz0_on_cto(){
  layz0.ctocnt++;
  layz0.rxlen= 0;
}
forward _layz1_on_cto();
public _layz1_on_cto(){
  layz1.ctocnt++;
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
