#pragma once
#define CMP_CC1101



#ifdef CMP_CC1101
	#ifdef ESP8266
		#define PIN_RECEIVE           5// D1
	#endif
#else
	#ifdef ESP8266
		#define PIN_RECEIVE            5
	#endif
#endif

#ifdef ESP8266
	#define PIN_LED                16
	#define PIN_SEND               4// D2  // gdo0Pin TX out
	#define DEBUG				   1
	#define ETHERNET_PRINT
#endif


#ifdef ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101
	#define SS					  8  
	#define PIN_MARK433			  4  // LOW -> 433Mhz | HIGH -> 868Mhz

#elif ARDUINO_ATMEGA328P_MINICUL  // 8Mhz 
	#define PIN_MARK433			  0
#endif
