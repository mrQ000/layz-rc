/*************************************************************************************************
	IO DRIVERS

	ioInit()
	
	public onButton( pressed, time_)
	
	setSysLed( rgb) - onboard system led
		rgb
			* 24bit color code 0xRRGGBB - script controlled color, where 0=black/off
			* -1 - controlled by runtime
			* -2 - totally off
	
	setLight( rgb) - peripheral rgb color stripe at GPIO 2/4/5 (R/G/B)
		rgb - 24bit color code 0xRRGGBB
	
	
*************************************************************************************************/


const {
	LED_STATIC  = 0,
	LED_BLINK	= 1,
	LED_FLICKER	= 2,
	LED_FLASH	= 3
}


/* @param rgb:s32
 *    0xRRGGBB - script controlled color, where 0=black/off
 *    -1 - controlled by runtime
 *    -2 - totally off
 */
setSysLed( rgb, mode=LED_STATIC, count=0){
  static script_= false;

  if (rgb>=0 && !script_) {
    script_= true;
    Led_Init( LED_MODE_SCRIPT);
  }
  if (rgb>0) {
	switch (mode) {
		case LED_FLICKER: Led_Flicker( count, rgb);	// static
		case LED_BLINK:   Led_Blink( count, rgb);	// static
		case LED_FLASH:   Led_Flash( count, rgb);	// static
		default: 		  Led_On( rgb);	// static
	}
  }
  else if (rgb==0) Led_Off();
  else if (rgb==-1) {
    script_= false;
    Led_Init( LED_MODE_INTERNAL);
  }
  else Led_Close();
}



new _light_r, _light_g, _light_b;
new _light_pattern= 0xffffffff; // STATIC

forward lightTask();
public lightTask(){

	static pdiv_= 0;
	static pmsk_= 1;			// rolling pattern mask
	static duty_=0x20;			// duty cycle counter, 8bit
	duty_= (duty_+0x40) & 0xff;

	// switch pattern after each duty cycle wrap around
	if (duty_==0x20) pdiv_++;
	if (pdiv_>1) {
		pdiv_= 0;
		pmsk_ <<= 1;
		if (!pmsk_) pmsk_= 1;
		if (!(_light_pattern & pmsk_)) {	// pattern disables light -> turn off until next enable
			rM2M_GpioSet( LIGHT_R, 0);
			rM2M_GpioSet( LIGHT_G, 0);
			rM2M_GpioSet( LIGHT_B, 0);
		}	  
	}
	if (_light_pattern & pmsk_) {
		rM2M_GpioSet( LIGHT_R, _light_r > duty_ ? 1 : 0);
		rM2M_GpioSet( LIGHT_G, _light_g > duty_ ? 1 : 0);
		rM2M_GpioSet( LIGHT_B, _light_b > duty_ ? 1 : 0);
	}	
}

setLight( rgb, mode=LED_STATIC){
	const {
		LIGHT_PATTERN_FLICKER = 0b01000110101110110001101011101001,
		LIGHT_PATTERN_BLINK	  = 0xffff0000,
		LIGHT_PATTERN_FLASH   = 0x30000000,
		LIGHT_PATTERN_STATIC  = 0xffffffff
	}
	
  new pattern;
  switch (mode){
  	case LED_FLICKER: pattern= LIGHT_PATTERN_FLICKER;
	case LED_BLINK:   pattern= LIGHT_PATTERN_BLINK;
	case LED_FLASH:   pattern= LIGHT_PATTERN_FLASH;
	default: 		  pattern= LIGHT_PATTERN_STATIC;
  }
  static on_= 0;    // lights are on
  static init_=0;
  if (!init_) {
    init_= 1;
    rM2M_GpioSet( LIGHT_R, 0);
    rM2M_GpioSet( LIGHT_G, 0);
    rM2M_GpioSet( LIGHT_B, 0);
    rM2M_GpioDir( LIGHT_R, RM2M_GPIO_OUTPUT);
    rM2M_GpioDir( LIGHT_G, RM2M_GPIO_OUTPUT);
    rM2M_GpioDir( LIGHT_B, RM2M_GPIO_OUTPUT);
  }

  static rgb_=0;
  if (rgb==rgb_ && pattern==_light_pattern) return;
  
  _light_r = (rgb >> 16) & 0xff;
  _light_g = (rgb >> 8) & 0xff;
  _light_b =  rgb & 0xff;
  _light_pattern= pattern;

  // save cpu energy - turn timer off if lights dark
  new on= rgb > 0;
  if (!on_ &&  on) setInterval( "lightTask", 3);
  if ( on_ && !on) {
    clearInterval( "lightTask");
    lightTask();      // execute lightTask one more time to turn off lights
  }
  on_= on;
}






forward onButton( pressed, time_);

forward buttonTask();
public buttonTask(){
  static init_=0;
  if (!init_){
    init_= 1;
    rM2M_GpioDir( BTN_LOCAL, RM2M_GPIO_INPUT);
  }
  static pressed_= 0;
  static time_= 0;
  new pressed= !rM2M_GpioGet( BTN_LOCAL);
  if (pressed_ != pressed) {
    pressed_ = pressed;
    onButton( pressed, time_);
    time_= 0;
  }
  time_++;
}


ioInit() {
	setInterval( "buttonTask", 100);
}
