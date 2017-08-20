// cc1101.h

#ifndef _CC1101_h
#define _CC1101_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include <EEPROM.h>
#include "output.h"

#ifdef ESP8266
	#include <SPI.h>
#endif

extern String cmdstring;



namespace cc1101 {
#if defined(ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101) 
  #ifndef ESP8266
		#define SS					  8  
  #endif
	#define PIN_MARK433			  4  // LOW -> 433Mhz | HIGH -> 868Mhz
#endif

	#define csPin	15   // CSN  out
	#define mosiPin 13   // MOSI out
	#define misoPin 12   // MISO in
	#define sckPin  14   // SCLK out	


	
	#define CC1100_WRITE_BURST    0x40
  #define CC1101_WRITE_SINGLE   0x00
  #define CC1100_READ_BURST     0xC0
  #define CC1101_READ_SINGLE    0x80
  #define CC1101_CONFIG         CC1101_READ_SINGLE
  #define CC1101_STATUS         CC1100_READ_BURST
	
	#define CC1100_FREQ2       0x0D  // Frequency control word, high byte
	#define CC1100_FREQ1       0x0E  // Frequency control word, middle byte
	#define CC1100_FREQ0       0x0F  // Frequency control word, low byte
	#define CC1100_PATABLE     0x3E  // 8 byte memory
	#define CC1100_IOCFG2      0x00  // GDO2 output configuration
	#define CC1100_PKTCTRL0    0x08  // Packet config register

	// Status registers - older version base on 0x30
  #define CC1101_PARTNUM      0xF0 // Chip ID
  #define CC1101_VERSION      0xF1 // Chip ID
  #define CC1100_RSSI         0xF4 // Received signal strength indication
	#define CC1100_MARCSTATE    0xF5 // Control state machine state
	 
	// Strobe commands
	#define CC1101_SRES     0x30  // reset
	#define CC1100_SFSTXON  0x31  // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
	#define CC1100_SCAL     0x33  // Calibrate frequency synthesizer and turn it off
	#define CC1100_SRX      0x34  // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
	#define CC1100_STX      0x35  // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1
	#define CC1100_SIDLE    0x36  // Exit RX / TX, turn off frequency synthesizer
	#define CC1100_SAFC     0x37  // Perform AFC adjustment of the frequency synthesizer
	#define CC1100_SFTX     0x3B  // Flush the TX FIFO buffer.
	#define CC1101_SNOP 	  0x3D	// 

  enum CC1101_MarcState {
    MarcStateSleep          = 0x00u
  , MarcStateIdle           = 0x01u
  , MarcStateXOff           = 0x02u
  , MarcStateVConnManCal    = 0x03u
  , MarcStateRegOnManCal    = 0x04u
  , MarcStateManCal         = 0x05u
  , MarcStateVConnFSWakeUp  = 0x06u
  , MarcStateRegOnFSWakeUp  = 0x07u
  , MarcStateStartCalibrate = 0x08u
  , MarcStateBWBoost        = 0x09u
  , MarcStateFSLock         = 0x0Au
  , MarcStateIfadCon        = 0x0Bu
  , MarcStateEndCalibrate   = 0x0Cu
  , MarcStateRx             = 0x0Du
  , MarcStateRxEnd          = 0x0Eu
  , MarcStateRxRst          = 0x0Fu
  , MarcStateTxRxSwitch     = 0x10u
  , MarcStateRxFifoOverflow = 0x11u
  , MarcStateFsTxOn         = 0x12u
  , MarcStateTx             = 0x13u
  , MarcStateTxEnd          = 0x14u
  , MarcStateRxTxSwitch     = 0x15u
  , MarcStateTxFifoUnerflow = 0x16u
  };
  
#ifdef ESP8266
	#define pinAsInput(pin) pinMode(pin, INPUT)
	#define pinAsOutput(pin) pinMode(pin, OUTPUT)
	#define pinAsInputPullUp(pin) pinMode(pin, INPUT_PULLUP)
	
	#ifndef digitalLow
		#define digitalLow(pin) digitalWrite(pin, LOW)
	#endif
	#ifndef digitalHigh
		#define digitalHigh(pin) digitalWrite(pin, HIGH)
	#endif
	#ifndef isHigh
		#define isHigh(pin) (digitalRead(pin) == HIGH)
	#endif
#endif

	#define wait_Miso()       while(isHigh(misoPin) ) { static uint8_t miso_count=255;delay(1); if(miso_count==0) return; miso_count--; }      // wait until SPI MISO line goes low 
    #define wait_Miso_rf()       while(isHigh(misoPin) ) { static uint8_t miso_count=255;delay(1); if(miso_count==0) return false; miso_count--; }      // wait until SPI MISO line goes low 

	#define cc1101_Select()   digitalLow(csPin)          // select (SPI) CC1101
	#define cc1101_Deselect() digitalHigh(csPin) 
	
	#define EE_CC1100_CFG        3
	#define EE_CC1100_CFG_SIZE   0x29
	#define EE_CC1100_PA         0x30  //  (EE_CC1100_CFG+EE_CC1100_CFG_SIZE)  // 2C
	#define EE_CC1100_PA_SIZE    8
	
	#define PATABLE_DEFAULT      0x84   // 5 dB default value for factory reset

	//------------------------------------------------------------------------------
	// Chip Status Byte
	//------------------------------------------------------------------------------

	// Bit fields in the chip status byte
	#define CC1100_STATUS_CHIP_RDYn_BM             0x80
	#define CC1100_STATUS_STATE_BM                 0x70
	#define CC1100_STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F

	// Chip states
	#define CC1100_STATE_IDLE                      0x00
	#define CC1100_STATE_RX                        0x10
	#define CC1100_STATE_TX                        0x20
	#define CC1100_STATE_FSTXON                    0x30
	#define CC1100_STATE_CALIBRATE                 0x40
	#define CC1100_STATE_SETTLING                  0x50
	#define CC1100_STATE_RX_OVERFLOW               0x60
	#define CC1100_STATE_TX_UNDERFLOW              0x70
	
#ifdef ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101
	uint8_t RADINOVARIANT = 0;            // Standardwert welcher je radinoVarinat ge�ndert wird
#endif
	static const uint8_t initVal[] PROGMEM = 
	{
		      // IDX NAME     RESET   COMMENT
		0x0D, // 00 IOCFG2    29     GDO2 as serial output
		0x2E, // 01 IOCFG1           Tri-State
		0x2D, // 02 IOCFG0    3F     GDO0 for input
		0x07, // 03 FIFOTHR   
		0xD3, // 04 SYNC1     
		0x91, // 05 SYNC0     
		0x3D, // 06 PKTLEN    0F
		0x04, // 07 PKTCTRL1  
		0x32, // 08 PKTCTRL0  45     
		0x00, // 09 ADDR     
		0x00, // 0A CHANNR   
		0x06, // 0B FSCTRL1   0F     152kHz IF Frquency
		0x00, // 0C FSCTRL0
		0x10, // 0D FREQ2     1E     Freq
		0xB0, // 0E FREQ1     C4     
		0x71, // 0F FREQ0     EC     
		0x57, // 10 MDMCFG4   8C     bWidth 325kHz
		0xC4, // 11 MDMCFG3   22     DataRate
		0x30, // 12 MDMCFG2   02     Modulation: ASK
		0x23, // 13 MDMCFG1   22     
		0xb9, // 14 MDMCFG0   F8     ChannelSpace: 350kHz
		0x00, // 15 DEVIATN   47     
		0x07, // 16 MCSM2     07     
		0x00, // 17 MCSM1     30     Bit 3:2  RXOFF_MODE:  Select what should happen when a packet has been received: 0 = IDLE  3 =  Stay in RX ####
		0x18, // 18 MCSM0     04     Calibration: RX/TX->IDLE
		0x14, // 19 FOCCFG    36     
		0x6C, // 1A BSCFG
		0x07, // 1B AGCCTRL2  03     42 dB instead of 33dB
		0x00, // 1C AGCCTRL1  40     
		0x90, // 1D AGCCTRL0  91     4dB decision boundery
		0x87, // 1E WOREVT1
		0x6B, // 1F WOREVT0
		0xF8, // 20 WORCTRL
		0x56, // 21 FREND1
		0x11, // 22 FREND0    16     0x11 for no PA ramping
		0xE9, // 23 FSCAL3    A9    E9 ?? 
		0x2A, // 24 FSCAL2    0A    
		0x00, // 25 FSCAL1    20    19 ??
		0x1F, // 26 FSCAL0    0D     
		0x41, // 27 RCCTRL1
		0x00, // 28 RCCTRL0
	};
  
	byte hex2int(byte hex) {    // convert a hexdigit to int    // Todo: printf oder scanf nutzen
		if (hex >= '0' && hex <= '9') hex = hex - '0';
		else if (hex >= 'a' && hex <= 'f') hex = hex - 'a' + 10;
		else if (hex >= 'A' && hex <= 'F') hex = hex - 'A' + 10;
		return hex;
		// printf ("%d\n",$hex) ??
	}

	void printHex2(const byte hex) {   // Todo: printf oder scanf nutzen
		if (hex < 16) {
			MSG_PRINT("0");
		}
		// char hexstr[2] = {0};
		//sprintf(hexstr, "%02X", hex);

		MSG_PRINT(hex, HEX);
	}

	uint8_t sendSPI(const uint8_t val) {				 // send byte via SPI
	#ifndef ESP8266
		SPDR = val;                                      // transfer byte via SPI
		while (!(SPSR & _BV(SPIF)));                     // wait until SPI operation is terminated
		return SPDR;
	#else
		return SPI.transfer(val);
	#endif
	}

	uint8_t cmdStrobe(const uint8_t cmd) {              // send command strobe to the CC1101 IC via SPI
		cc1101_Select();                                // select CC1101
		wait_Miso_rf();                                 // wait until MISO goes low
		uint8_t ret = sendSPI(cmd);                     // send strobe command
		wait_Miso_rf();                                 // wait until MISO goes low
		cc1101_Deselect();                              // deselect CC1101
		return ret;										// Chip Status Byte
	}

	uint8_t readReg(const uint8_t regAddr, const uint8_t regType) {       // read CC1101 register via SPI
		cc1101_Select();                                // select CC1101
		wait_Miso_rf();                                    // wait until MISO goes low
		sendSPI(regAddr | regType);                     // send register address
		uint8_t val = sendSPI(0x00);                    // read result
		cc1101_Deselect();                              // deselect CC1101
		return val;
	}

	void writeReg(const uint8_t regAddr, const uint8_t val) {     // write single register into the CC1101 IC via SPI
		cc1101_Select();                                // select CC1101
		wait_Miso();                                    // wait until MISO goes low
		sendSPI(regAddr);                               // send register address
		sendSPI(val);                                   // send value
		cc1101_Deselect();                              // deselect CC1101
	}

	void readPatable(void) {
		uint8_t PatableArray[8];
		// das PatableArray wird zum zwischenspeichern der PATABLE verwendet,
		// da ich mir nicht sicher bin ob es timing maessig passt, wenn es nach jedem sendSPI(0x00) eine kurze Pause beim msgprint gibt.
		
		cc1101_Select();                                // select CC1101
		wait_Miso();                                    // wait until MISO goes low
		sendSPI(CC1100_PATABLE | CC1100_READ_BURST);    // send register address
		for (uint8_t i = 0; i < 8; i++) {
			PatableArray[i] = sendSPI(0x00);        // read result
		}
		cc1101_Deselect();

		for (uint8_t i = 0; i < 8; i++) {
			printHex2(PatableArray[i]);
			MSG_PRINT(" ");
		}
		MSG_PRINTLN("");
	}

	void writePatable(void) {
		cc1101_Select();                                // select CC1101
		wait_Miso();                                    // wait until MISO goes low
		sendSPI(CC1100_PATABLE | CC1100_WRITE_BURST);   // send register address
		for (uint8_t i = 0; i < 8; i++) {
			sendSPI(EEPROM.read(EE_CC1100_PA+i));                     // send value
		}
			cc1101_Deselect();
	}


	void readCCreg(const uint8_t reg) {   // read CC11001 register
		uint8_t var;
		uint8_t hex;
		uint8_t n;

		if (cmdstring.charAt(3) == 'n' && isHexadecimalDigit(cmdstring.charAt(4))) {   // C<reg>n<anz>  gibt anz+2 fortlaufende register zurueck
		   hex = (uint8_t)cmdstring.charAt(4);
		   n = hex2int(hex);
		   if (reg < 0x2F) {
			  MSG_PRINT("C");
			  printHex2(reg);
			  MSG_PRINT("n");
			  n += 2;
			  printHex2(n);
			  MSG_PRINT("=");
			  for (uint8_t i = 0; i < n; i++) {
				 var = readReg(reg + i, CC1101_CONFIG);
				 printHex2(var);
			  }
			  MSG_PRINTLN("");
		   }
		} else {
		if (reg < 0x3E) {
		  if (reg < 0x2F) {
			 var = readReg(reg, CC1101_CONFIG);
		  } else {
			 var = readReg(reg, CC1101_STATUS);
		  }
		  MSG_PRINT("C");
		  printHex2(reg);
		  MSG_PRINT(" = ");
		  printHex2(var);
		  MSG_PRINTLN("");
		} else if (reg == 0x3E) {                   // patable
		  MSG_PRINT(F("C3E = "));
		  readPatable();
		} else if (reg == 0x99) {                   // alle register
		 for (uint8_t i = 0; i < 0x2f; i++) {
		   if (i == 0 || i == 0x10 || i == 0x20) {
			 if (i > 0) {
			   MSG_PRINT(" ");
			 }
			 MSG_PRINT(F("ccreg "));
			 printHex2(i);
			 MSG_PRINT(F(": "));
		   }
		   var = readReg(i, CC1101_CONFIG);
		   printHex2(var);
		   MSG_PRINT(" ");
		 }
			MSG_PRINTLN("");
		}
		}
	}

	void commandStrobes(void) {
		uint8_t hex;
		uint8_t reg;
		uint8_t val;
		uint8_t val1;

		if (isHexadecimalDigit(cmdstring.charAt(3))) {
			hex = (uint8_t)cmdstring.charAt(3);
			reg = hex2int(hex) + 0x30;
			if (reg < 0x3e) {
				 val = cmdStrobe(reg);
				 delay(1);
				 val1 = cmdStrobe(0x3D);        //  No operation. May be used to get access to the chip status byte.
				 MSG_PRINT(F("cmdStrobeReg "));
				 printHex2(reg);
				 MSG_PRINT(F(" chipStatus "));
				 val = val >> 4;
				 MSG_PRINT(val, HEX);
				 MSG_PRINT(F(" delay1 "));
				 val = val1 >> 4;
				 MSG_PRINT(val, HEX);
				 MSG_PRINTLN("");
			 }
		 }
	}

	void writeCCreg(uint8_t reg, uint8_t var) {    // write CC11001 register
		if (reg > 1 && reg < 0x40) {
			   writeReg(reg-2, var);
			   MSG_PRINT("W");
			   printHex2(reg);
			   printHex2(var);
			   MSG_PRINTLN("");
		}
	}

	void writeCCpatable(uint8_t var) {           // write 8 byte to patable (kein pa ramping)
		for (uint8_t i = 0; i < 8; i++) {
			if (i == 1) {
				EEPROM.write(EE_CC1100_PA + i, var);
			} else {
				EEPROM.write(EE_CC1100_PA + i, 0);
			}
		}
		writePatable();
	}

	void ccFactoryReset() {
		for (uint8_t i = 0; i<sizeof(initVal); i++) {
        		EEPROM.write(EE_CC1100_CFG + i, pgm_read_byte(&initVal[i]));
		}
		for (uint8_t i = 0; i < 8; i++) {
			if (i == 1) {
				EEPROM.write(EE_CC1100_PA + i, PATABLE_DEFAULT);
			} else {
				EEPROM.write(EE_CC1100_PA + i, 0);
			}
		}
		MSG_PRINTLN("ccFactoryReset done");  
	}

  uint8_t chipVersion() { return readReg(CC1101_VERSION, CC1101_READ_SINGLE); };
	bool checkCC1101() {

		uint8_t partnum = readReg(CC1101_PARTNUM, CC1101_READ_SINGLE);  // Partnum
		uint8_t version = chipVersion();  // Version
		DBG_PRINT("CCVersion=");	DBG_PRINTLN("0x" + String(version, HEX));
		DBG_PRINT("CCPartnum=");	DBG_PRINTLN("0x" + String(partnum, HEX));

		//checks if valid Chip ID is found. Usualy 0x03 or 0x14. if not -> abort
		if (version == 0x00 || version == 0xFF)
		{
			DBG_PRINTLN(F("no CC11xx found!"));
			DBG_PRINTLN();
			return false;  // Todo: power down SPI etc
		}
		return true;
	}



	inline void setup()
	{
		#ifndef ESP8266
		pinAsOutput(sckPin);
		pinAsOutput(mosiPin);
		pinAsInput(misoPin);
		#endif
		pinAsOutput(csPin);                    // set pins for SPI communication
		
		#ifdef ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101
		pinAsInputPullUp(PIN_MARK433);
		#endif
		//// �nderungsbeginn  ---> 

#ifndef ESP8266
		SPCR = _BV(SPE) | _BV(MSTR);               // SPI speed = CLK/4
		digitalHigh(csPin);                 // SPI init
		digitalHigh(sckPin);
		digitalLow(mosiPin);
#else
		SPI.setDataMode(SPI_MODE0);
		SPI.setBitOrder(MSBFIRST);
		SPI.begin();
		SPI.setClockDivider(SPI_CLOCK_DIV4);
#endif
		pinAsInput(PIN_RECEIVE);    // gdo2
		pinAsOutput(PIN_SEND);      // gdo0Pi, sicherheitshalber bis zum CC1101 init erstmal input   
	}

	uint8_t getRSSI()
	{
		return readReg(CC1100_RSSI, CC1101_STATUS);// Pruefen ob Umwandung von uint to int den richtigen Wert zurueck gibt
	}
	
	inline void setIdleMode()
	{
		cmdStrobe(CC1100_SIDLE);                             // Idle mode
		delay(1);
	}

	uint8_t currentMode() {
		return readReg(CC1100_MARCSTATE, CC1100_READ_BURST);
	}
	
	void setReceiveMode()
	{
		setIdleMode();
		uint8_t maxloop = 0xff;

		while (maxloop-- &&	(cmdStrobe(CC1100_SRX) & CC1100_STATUS_STATE_BM) != CC1100_STATE_RX) // RX enable
			delay(1);
		if (maxloop == 0 )		DBG_PRINTLN("CC1101: Setting RX failed");

	}

	void setTransmitMode()
	{
		cmdStrobe(CC1100_SFTX);   // wird dies benoetigt? Wir verwenden kein FIFO
		setIdleMode();
		uint8_t maxloop = 0xff;
		while (maxloop-- && (cmdStrobe(CC1100_STX) & CC1100_STATUS_STATE_BM) != CC1100_STATE_TX)  // TX enable
			delay(1);
		if (maxloop == 0) DBG_PRINTLN("CC1101: Setting TX failed");
	}

	void CCinit(void) {                              // initialize CC1101

		cc1101_Deselect();                                  // some deselect and selects to init the cc1101
		delayMicroseconds(30);

		// Begin of power on reset
		cc1101_Select();
		delayMicroseconds(30);

		cc1101_Deselect();
		delayMicroseconds(45);
		
		DBG_PRINTLN("SRES Started");
		cmdStrobe(CC1101_SRES);                               // send reset
		DBG_PRINTLN("POR Done");
		delay(10);

		cc1101_Select();
		
		sendSPI(CC1100_WRITE_BURST);
		for (uint8_t i = 0; i<sizeof(initVal); i++) {              // write EEPROM value to cc11001
			sendSPI(EEPROM.read(EE_CC1100_CFG + i));
		}
		cc1101_Deselect();
		delayMicroseconds(10);            // ### todo: welcher Wert ist als delay sinnvoll? ###

		writePatable();                                 // write PatableArray to patable reg

		delay(1);
		setReceiveMode();
	}

	bool regCheck()
	{
		
		DBG_PRINT("CC1100_PKTCTRL0="); DBG_PRINT(readReg(CC1100_PKTCTRL0, CC1101_CONFIG));
		DBG_PRINT(" vs EEPROM PKTCTRL0="); DBG_PRINTLN(initVal[CC1100_PKTCTRL0]);

		DBG_PRINT("C1100_IOCFG2="); DBG_PRINT(readReg(CC1100_IOCFG2, CC1101_CONFIG));
		DBG_PRINT(" vs EEPROM IOCFG2="); DBG_PRINTLN(initVal[CC1100_IOCFG2]);
		
		return (readReg(CC1100_PKTCTRL0, CC1101_CONFIG) == initVal[CC1100_PKTCTRL0]) && (readReg(CC1100_IOCFG2, CC1101_CONFIG) == initVal[CC1100_IOCFG2]);
	}

}

#endif
