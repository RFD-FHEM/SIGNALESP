

#define PROGNAME               "RF_RECEIVER-ESP"
#define PROGVERS               "3.3.1-rc2"
#define VERSION_1               0x33
#define VERSION_2               0x1d

#define CMP_CC1101

#ifdef CMP_CC1101
  #include <pins_arduino.h> // prevent travis errors
	#define PIN_RECEIVE            D1
#else
	#define PIN_RECEIVE            2
#endif

#define PIN_LED                16
#define PIN_SEND               D2  // gdo0Pin TX out
#define BAUDRATE               115200
#define FIFO_LENGTH			   200
#define DEBUG				   1
#define _DEBUG_DEV_SERIAL

#ifdef _DEBUG_DEV_SERIAL
//  #define _DEBUG_DEV_SERIAL_SEND_DELAYED
//  #define _CC1101_DEBUG_CONFIG
#endif


#define _USE_WRITE_BUFFER

#ifdef _USE_WRITE_BUFFER
  const size_t writeBufferSize = 256;
  size_t writeBufferCurrent = 0;
  uint8_t writeBuffer[writeBufferSize+1]; // one extra buffer for \n

  //#define _USE_WRITE_BUFFER_DEBUG 1
  //#define _USE_WRITE_BUFFER_STATS
  
  #ifdef _USE_WRITE_BUFFER_STATS
    size_t writeCalls = 0;
    unsigned long firstWriteCall;
  #endif  // _USE_WRITE_BUFFER_STATS
#endif  // _USE_WRITE_BUFFER


#define ETHERNET_PRINT
#include <FS.h>   
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>

WiFiServer Server(23);  //  port 23 = telnet
WiFiClient serverClient;

#include <output.h>
#include <bitstore.h>  // Die wird aus irgend einem Grund zum Compilieren benoetigt.
#include <SimpleFIFO.h>

SimpleFIFO<int, FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
#include <signalDecoder.h>
#include <FastDelegate.h> // prevent travis errors
SignalDetectorClass musterDec;


#ifdef CMP_CC1101
  #include "cc1101.h"
  #include <SPI.h>      // prevent travis errors
#endif

#define pulseMin  90
volatile bool blinkLED = false;
String cmdstring = "";
volatile unsigned long lastTime = micros();

#define digitalLow(P) digitalWrite(P,LOW)
#define digitalHigh(P) digitalWrite(P,HIGH)
#define isHigh(P) (digitalRead(P) == HIGH)
#define isLow(P) (digitalRead(P) == LOW)
#define digitalState(P)((uint8_t)isHigh(P))

extern "C" {
#include "user_interface.h"
}

os_timer_t cronTimer;


bool hasCC1101 = false;

// EEProm Addresscommands
#define EE_MAGIC_OFFSET      0
#define addr_features EE_MAGIC_OFFSET+2
#define MAX_SRV_CLIENTS 2



//void handleInterrupt();
void enableReceive();
void disableReceive();
void serialEvent();
void cronjob(void *pArg);
int freeRam();
void changeReciver();
void HandleCommand();
bool command_available = false;
unsigned long getUptime();
void getConfig();
void enDisPrint(bool enDis);
void getPing();
void configCMD();
void storeFunctions(const int8_t ms = 1, int8_t mu = 1, int8_t mc = 1);
void getFunctions(bool *ms, bool *mu, bool *mc);
uint8_t rssiCallback() { return 0; }; // Dummy return if no rssi value can be retrieved from receiver
//uint16_t writeCallback(const uint8_t *buf,uint8_t len);

//default custom static IP
char static_ip[16] = "10.22.0.200";
char static_gw[16] = "0.0.0.0";
char static_sn[16] = "255.255.255.0";

bool startWPS() {
	// from https://gist.github.com/copa2/fcc718c6549721c210d614a325271389
	// wpstest.ino
	Serial.println("WPS config start");
	WiFi.disconnect();
	WiFi.mode(WIFI_STA); // WPS only works in station mode
	
	delay(1000);
	bool wpsSuccess = WiFi.beginWPSConfig();
	if (wpsSuccess) {
		// Well this means not always success :-/ in case of a timeout we have an empty ssid
		String newSSID = WiFi.SSID();
		if (newSSID.length() > 0) {
			// WPSConfig has already connected in STA mode successfully to the new station. 
			Serial.printf("WPS finished. Connected successfull to SSID '%s'\n", newSSID.c_str());
		}
		else {
			wpsSuccess = false;
		}
	}
	return wpsSuccess;
}


//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback() {
	DBG_PRINTLN("Should save config");
	shouldSaveConfig = true;
}

void setup() {
	//ESP.wdtEnable(2000);

  Serial.begin(115200);
//  Serial.setDebugOutput(true);
  while (!Serial)
    delay(90);

  Serial.println("\n\n");

  pinMode(PIN_RECEIVE, INPUT);
  pinMode(PIN_LED, OUTPUT);
  
#ifdef CMP_CC1101
  cc1101::setup();
#endif

  initEEPROM();

  // register write callback
  musterDec.registerWriteCallback(writeCallback);
  
#ifdef CMP_CC1101
  cc1101::CCinit();
  hasCC1101 = cc1101::checkCC1101();
  if (hasCC1101) {
      DBG_PRINTLN("CC1101 found (rev. 0" + String(cc1101::getRevision(), HEX) + ")");
      musterDec.setRSSICallback(&cc1101::getRSSI);                    // Provide the RSSI Callback
  }
#endif  

/*
  #ifdef DEBUG
    Serial.printf("\nTry connecting to WiFi with SSID '%s'\n", WiFi.SSID().c_str());
  #endif

	if (WiFi.SSID().length() > 0 )
	{

		WiFi.mode(WIFI_STA);
		WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str()); // reading data from EPROM, 
		while (WiFi.status() == WL_DISCONNECTED) {          // last saved credentials
			delay(500);
			Serial.print(".");
		}
	}
  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
  #ifdef DEBUG 
	   Serial.printf("\nConnected successful to SSID '%s'\n", WiFi.SSID().c_str());
  #endif
  }  else {
	Serial.printf("\nCould not connect to WiFi. state='%d'\n", status);
	Serial.println("Please press WPS button on your router");
	delay(5000);
	if (!startWPS()) {
		Serial.println("Failed to connect with WPS :-(");
	}
	else 
	{
		WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str()); // reading data from EPROM, 
		while (WiFi.status() == WL_DISCONNECTED) {          // last saved credentials
			delay(500);
			Serial.print("."); // show wait for connect to AP
				
		}
		#ifdef DEBUG
		Serial.print("\nReady! Use 'telnet ");
		Serial.print(WiFi.localIP());
		Serial.println(" port 23' to connect");
		#endif

	}


  }
 */

	DBG_PRINTLN("mounting FS...");

	if (SPIFFS.begin()) {
		DBG_PRINTLN("mounted file system");
		if (SPIFFS.exists("/config.json")) {
			//file exists, reading and loading
			DBG_PRINTLN("reading config file");
			File configFile = SPIFFS.open("/config.json", "r");
			if (configFile) {
				DBG_PRINTLN("opened config file");
				size_t size = configFile.size();
				// Allocate a buffer to store contents of the file.
				std::unique_ptr<char[]> buf(new char[size]);

				configFile.readBytes(buf.get(), size);
				DynamicJsonBuffer jsonBuffer;
				JsonObject& json = jsonBuffer.parseObject(buf.get());
				json.printTo(Serial);
				if (json.success()) {
					DBG_PRINTLN("\nparsed json");
					if (json["ip"]) {
						DBG_PRINTLN("setting custom ip from config");
						//static_ip = json["ip"];
						strcpy(static_ip, json["ip"]);
						strcpy(static_gw, json["gateway"]);
						strcpy(static_sn, json["subnet"]);
						DBG_PRINTLN(static_ip);
					}
					else {
						DBG_PRINTLN("no custom ip in config");
					}
				}
				else {
					DBG_PRINTLN("failed to load json config");
				}
			}
		}
	}
	else {
		DBG_PRINTLN("failed to mount FS");
	}
	//end read

	WiFiManager wifiManager;
  //  wifiManager.setBreakAfterConfig(true);
  //reset settings - for testing
  //wifiManager.resetSettings();

  //tries to connect to last known settings
  //if it does not connect it starts an access point with the specified name
  //here  "NodeDuino" with no password
  //and goes into a blocking loop awaiting configuration
/*
  // old wifi crap
  if (!wifiManager.autoConnect("NodeDuino")) {
    Serial.println("failed to connect, we should reset as see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
  }
*/
  // new wifi crap
  wifiManager.setConfigPortalTimeout(60);
	wifiManager.setSaveConfigCallback(saveConfigCallback);

	IPAddress _ip, _gw, _sn;

	_ip.fromString(static_ip);
	_gw.fromString(static_gw);
	_sn.fromString(static_sn);
	wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
  // end wifi crap


	if (!wifiManager.startConfigPortal()) {

		Serial.println("failed to connect, we now enter WPS Mode");
		delay(3000);


			Serial.printf("\nCould not connect to WiFi. state='%d'\n", WiFi.status());
			Serial.println("Please press WPS button on your router");
			delay(5000);
			if (!startWPS()) {
				Serial.println("Failed to connect with WPS, will restart ESP now :-(");
				delay(3500);
				while (1);
				//ESP.restart();
				//ESP.reset();
			}

		delay(5000);
	}

	if (WiFi.status() == WL_CONNECTED) {          // last saved credentials

	  //if you get here you have connected to the WiFi
		Serial.println("connected...)");

		Serial.print("local ip: ");
		Serial.println(WiFi.localIP());

		if (shouldSaveConfig) {
			DBG_PRINTLN("saving config");
			DynamicJsonBuffer jsonBuffer;
			JsonObject& json = jsonBuffer.createObject();

			json["ip"] = WiFi.localIP().toString();
			json["gateway"] = WiFi.gatewayIP().toString();
			json["subnet"] = WiFi.subnetMask().toString();

			File configFile = SPIFFS.open("/config.json", "w");
			if (!configFile) {
				DBG_PRINTLN("failed to open config file for writing");
			}

			json.prettyPrintTo(Serial);
			json.printTo(configFile);
			configFile.close();
			//end save
		}

	}
	Server.begin();  // telnet server
	Server.setNoDelay(true);



	os_timer_disarm(&cronTimer);
	os_timer_setfn(&cronTimer, cronjob, NULL);
	os_timer_arm(&cronTimer, 31, true);

#ifdef CMP_CC1101
	if (!hasCC1101 || cc1101::regCheck()) {
#endif
	
	enableReceive();
    DBG_PRINTLN(F("receiver enabled"));
#ifdef CMP_CC1101
  } else {
    DBG_PRINTLN(F("cc1101 is not correctly set. Please do a factory reset via command e"));
  }
#endif

	cmdstring.reserve(40);

}

void ICACHE_RAM_ATTR cronjob(void *pArg) {
	digitalWrite(PIN_LED, blinkLED);
	blinkLED = false;
}


uint8_t fifousage = 0;

void loop() {
	static int aktVal = 0;
	bool state;
	serialEvent();
	ethernetEvent();

	if (command_available) {
		command_available = false;
		HandleCommand();
		if (!command_available) { cmdstring = ""; }
		blinkLED = true;
	}
	
	if (fifousage < FiFo.count())
	  fifousage = FiFo.count();
  while (FiFo.count()>0) { //Puffer auslesen und an Dekoder uebergeben
  		aktVal = FiFo.dequeue();
	  	state = musterDec.decode(&aktVal);
		  if (state) blinkLED = true; //LED blinken, wenn Meldung dekodiert
//      if (FiFo.count()<120) yield();
	}
  
#ifdef _DEBUG_DEV_SERIAL
  if (Serial.available()) {
    unsigned char c = Serial.read();
//    if (c != char(10))
//      Serial.println(".");

    int8_t rssi;
    switch(c) {
      case 'c':
#ifdef CMP_CC1101
        Serial.println("marc: 0x" + String(cc1101::currentMode(), HEX));
#endif
        Serial.println("fifo: " + String(FiFo.count()) + ", max. " + String(fifousage));
#ifdef _CC1101_DEBUG_CONFIG
        cc1101::dumpConfigRegister();
#endif
        break;
      case 'd':
        dumpEEPROM();
        break;
#ifdef CMP_CC1101
      case 'r':
        rssi = cc1101::getRSSI();
        Serial.printf("rssi: 0x%2X %d\n", (uint8_t)rssi, rssi);
        break;
#endif
      case 's':
#ifndef _DEBUG_DEV_SERIAL_SEND_DELAYED
        MSG_PRINTLN("Test Test Test");
#else
        ESP.wdtDisable();
        MSG_PRINT("Test Test ");
        delayMicroseconds(1000*1000);
        MSG_PRINTLN("1s delay Test");
        delayMicroseconds(2000*1000);
        MSG_PRINTLN("2s delay without yield()");
        ESP.wdtEnable(1000);
#endif  // _DEBUG_DEV_SERIAL_SEND_DELAYED
        break;
      case 'u':
        Serial.println("uptime: " + uptime());
        break;
      case 'w':
        unsigned long start = micros();
        for (byte i=0; i<12; i++) // 240 Zeichen
          musterDec.write("Test Test Test Test ");
        musterDec.write("Test Test 256=>\n<= ");
        musterDec.write("Test ");
        musterDec.write(MSG_END);
        musterDec.write("\n");
        Serial.println("elapsed time: " + String(micros() - start) + " us");
        break;
    }
  }
#endif	// _DEBUG_DEV_SERIAL
}



//========================= Pulseauswertung ================================================
void ICACHE_RAM_ATTR  handleInterrupt() {
	const unsigned long Time = micros();
	//const bool state = digitalRead(PIN_RECEIVE);
	const unsigned long  duration = Time - lastTime;
	lastTime = Time;
	if (duration >= pulseMin) {//kleinste zulaessige Pulslaenge
		int sDuration;
		if (duration < maxPulse) {//groesste zulaessige Pulslaenge, max = 32000
			sDuration = int(duration); //das wirft bereits hier unnoetige Nullen raus und vergroessert den Wertebereich
		}
		else {
			sDuration = maxPulse; // Maximalwert set to maxPulse defined in lib.
		}
		if (isHigh(PIN_RECEIVE)) { // Wenn jetzt high ist, dann muss vorher low gewesen sein, und dafuer gilt die gemessene Dauer.
			sDuration = -sDuration;
		}
		FiFo.enqueue(sDuration);
	} // else => trash
}

void enableReceive() {
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVE), handleInterrupt, CHANGE);
  
  #ifdef CMP_CC1101
    if (hasCC1101)
      cc1101::setReceiveMode();
  #endif
}

void disableReceive() {
  detachInterrupt(digitalPinToInterrupt(PIN_RECEIVE));
  
  #ifdef CMP_CC1101
    if (hasCC1101)
      cc1101::setIdleMode();
  #endif
	FiFo.flush();
}


//============================== Write callback =========================================
#ifdef _USE_WRITE_BUFFER
#ifdef _USE_WRITE_BUFFER_DEBUG
void dumpBuffer() {
//#if _USE_WRITE_BUFFER_DEBUG < 2
//    return;
//#endif
    
  Serial.println("buffer: ");
  for (size_t idx=0; idx<writeBufferCurrent+1; idx++) {
    Serial.printf("%02X ", writeBuffer[idx]);
    if (idx % 16 == 15)
      Serial.println();
  }
  Serial.println();
}
#endif  // _USE_WRITE_BUFFER_DEBUG

uint8_t *memchr2(const uint8_t *ptr, uint8_t ch, size_t size) {
  for (int i = 0; i < size; i++)
    if (*ptr++ == ch)
      return (uint8_t*)ptr;
  return NULL;
}
#endif  // _USE_WRITE_BUFFER

size_t writeCallback(const uint8_t *buf, size_t len)
{
  size_t res = 0;
#ifdef _USE_WRITE_BUFFER_STATS
  if (writeCalls == 0)
    firstWriteCall = micros();
  writeCalls++;
#endif  // _USE_WRITE_BUFFER_STATS
#ifdef _USE_WRITE_BUFFER
  size_t  remain = len, bufpos = 0, wrote = 0;

#ifdef _USE_WRITE_BUFFER_DEBUG
    Serial.println("");
#endif  // _USE_WRITE_BUFFER_DEBUG
  while (remain > 0) {
    size_t copy = (remain > writeBufferSize - writeBufferCurrent ? writeBufferSize - writeBufferCurrent : remain);
#ifdef _USE_WRITE_BUFFER_DEBUG
    Serial.println("writeBufferCurrent: " + String(writeBufferCurrent) + ", remain: " + String(remain) + ", bufpos: " + String(bufpos) + ", copy: " + String(copy));
//    dumpBuffer();
#endif  // _USE_WRITE_BUFFER_DEBUG

    // newline
    void *newLine;
    size_t newLinePos = 0;
    if ((newLine = memchr2(&buf[bufpos], 10, copy)) != NULL) {
      newLinePos = (size_t)newLine - (size_t)&buf[bufpos];
#ifdef _USE_WRITE_BUFFER_DEBUG
      Serial.println("writeCallback: newline @" + String(newLinePos));
#endif  // _USE_WRITE_BUFFER_DEBUG
      copy = newLinePos;
    }
     
    // copy to buffer
    memcpy(&writeBuffer[writeBufferCurrent], &buf[bufpos], copy);
    writeBufferCurrent += copy;
    bufpos += copy;
    remain -= copy;
    res += copy;

    // newline detected - force send
    if (newLinePos > 0)
      wrote += writeFromBuffer(writeBufferCurrent);

    // buffer full
    if (writeBufferCurrent == writeBufferSize) {
      wrote += writeFromBuffer(writeBufferCurrent);
    }
  }
#ifdef _USE_WRITE_BUFFER_DEBUG
//  dumpBuffer();
  Serial.println("writeBufferCurrent: " + String(writeBufferCurrent) + ", wrote: " + String(wrote));
#endif  // _USE_WRITE_BUFFER_DEBUG
#else // _USE_WRITE_BUFFER
  if (serverClient && serverClient.connected())
    res = serverClient.write(buf, len);
#endif // _USE_WRITE_BUFFER
  
  return res;
}

#ifdef _USE_WRITE_BUFFER
size_t writeFromBuffer(size_t len) {
  size_t res = 0;
  unsigned long lastWriteCall;

  if (serverClient && serverClient.connected())
    res = serverClient.write(&writeBuffer[0], len);

//  if (res > 0) {
//#ifdef _USE_WRITE_BUFFER_DEBUG
//    dumpBuffer();
//#endif  // _USE_WRITE_BUFFER_DEBUG
//  }

  writeBufferCurrent = 0;

#ifdef _USE_WRITE_BUFFER_STATS
  if (res > 0) {
    lastWriteCall = micros();
    Serial.println("wrote: " + String(res) + " bytes, calls: " + writeCalls + ", elaps: " + String(lastWriteCall - firstWriteCall) + " us");
    writeCalls = 0;
  }
#endif  // _USE_WRITE_BUFFER_STATS
  
  return res;
}
#endif  // _USE_WRITE_BUFFER

//============================== IT_Send =========================================

//================================= RAW Send ======================================
void send_raw(const uint8_t startpos, const uint16_t endpos, const int16_t *buckets, String *source = &cmdstring)
{
	uint8_t index = 0;
	unsigned long stoptime = micros();
	bool isLow;
	uint16_t dur;
	for (uint16_t i = startpos; i <= endpos; i++)
	{
		//MSG_PRINT(cmdstring.substring(i,i+1));
		index = source->charAt(i) - '0';
		//MSG_PRINT(index);
		isLow = buckets[index] >> 15;
		dur = abs(buckets[index]); 		//isLow ? dur = abs(buckets[index]) : dur = abs(buckets[index]);

		while (stoptime > micros()) {
			yield();
			
		}
		isLow ? digitalLow(PIN_SEND) : digitalHigh(PIN_SEND);
		stoptime += dur;
	}
	while (stoptime > micros()) {
		yield();
	}
	//MSG_PRINTLN("");

}
//SM;R=2;C=400;D=AFAFAF;

void send_mc(const uint8_t startpos, const uint8_t endpos, const int16_t clock)
{
	int8_t b;
	char c;
	//digitalHigh(PIN_SEND);
	//delay(1);
	uint8_t bit;

	unsigned long stoptime = micros();
	for (uint8_t i = startpos; i <= endpos; i++) {
		c = cmdstring.charAt(i);
		b = ((byte)c) - (c <= '9' ? 0x30 : 0x37);

		for (bit = 0x8; bit>0; bit >>= 1) {
			for (byte i = 0; i <= 1; i++) {
				if ((i == 0 ? (b & bit) : !(b & bit)))
					digitalLow(PIN_SEND);
				else
					digitalHigh(PIN_SEND);

				stoptime += clock;
				while (stoptime > micros())
					yield();
			}

		}

	}
	// MSG_PRINTLN("");
}



bool split_cmdpart(int16_t *startpos, String *msg_part)
{
	int16_t endpos = 0;
	//startpos=cmdstring.indexOf(";",startpos);   			 // search first  ";"
	endpos = cmdstring.indexOf(";", *startpos);     			 // search next   ";"

	if (endpos == -1 || *startpos == -1) return false;
	*msg_part = cmdstring.substring(*startpos, endpos);
	*startpos = endpos + 1;    // Set startpos to endpos to extract next part
	return true;
}
// SC;R=4;SM;C=400;D=AFFFFFFFFE;SR;P0=-2500;P1=400;D=010;SM;D=AB6180;SR;D=101;
// SC;R=4;SM;C=400;D=FFFFFFFF;SR;P0=-400;P1=400;D=101;SM;D=AB6180;SR;D=101;
// SR;R=3;P0=1230;P1=-3120;P2=-400;P3=-900;D=030301010101010202020202020101010102020202010101010202010120202;
// SM;C=400;D=AAAAFFFF;
// SR;R=10;P0=-2000;P1=-1000;P2=500;P3=-6000;D=2020202021212020202121212021202021202121212023;

struct s_sendcmd {
	int16_t sendclock = 0;
	uint8_t type;
	uint8_t datastart = 0;
	uint16_t dataend = 0;
	int16_t buckets[6];
	uint8_t repeats = 1;
};


void send_cmd()
{
#define combined 0
#define manchester 1
#define raw 2

	String msg_part;
	msg_part.reserve(30);
	uint8_t repeats = 1;  // Default is always one iteration so repeat is 1 if not set
						  //uint8_t type;
	int16_t start_pos = 0;
	//int16_t buckets[6]={};
	uint8_t counter = 0;
	//uint16_t sendclock;
	bool extraDelay = true;

	s_sendcmd command[5];

	uint8_t ccParamAnz = 0;   // Anzahl der per F= uebergebenen cc1101 Register
	uint8_t ccReg[4];
	uint8_t val;

	disableReceive();

	uint8_t cmdNo = 255;


	while (split_cmdpart(&start_pos, &msg_part))
	{
		DBG_PRINTLN(msg_part);
		if (msg_part.charAt(0) == 'S')
		{
			if (msg_part.charAt(1) == 'C')  // send combined informatio flag
			{
				//type=combined;
				//cmdNo=255;
				cmdNo++;
				//command[cmdNo].repeats = 0;
				command[cmdNo].type = combined;
				extraDelay = false;
			}
			else if (msg_part.charAt(1) == 'M') // send manchester
			{
				//type=manchester;
				cmdNo++;
				//command[cmdNo].repeats = 0;
				command[cmdNo].type = manchester;
				DBG_PRINTLN("Adding manchester");

			}
			else if (msg_part.charAt(1) == 'R') // send raw
			{
				//type=raw;
				cmdNo++;
				//command[cmdNo].repeats = 0;
				command[cmdNo].type = raw;
				DBG_PRINTLN("Adding raw");
				extraDelay = false;
			}
		}
		else if (msg_part.charAt(0) == 'P' && msg_part.charAt(2) == '=') // Do some basic detection if data matches what we expect
		{
			counter = msg_part.substring(1, 2).toInt(); // extract the pattern number
														//buckets[counter]=  msg_part.substring(3).toInt();
			command[cmdNo].buckets[counter] = msg_part.substring(3).toInt();
			DBG_PRINTLN("Adding bucket");

		}
		else if (msg_part.charAt(0) == 'R' && msg_part.charAt(1) == '=') {
			command[cmdNo].repeats = msg_part.substring(2).toInt();
			DBG_PRINT("Adding repeats: "); DBG_PRINTLN(command[cmdNo].repeats);


		}
		else if (msg_part.charAt(0) == 'D') {
			command[cmdNo].datastart = start_pos - msg_part.length() + 1;
			command[cmdNo].dataend = start_pos - 2;
			DBG_PRINT("locating data start:");
			DBG_PRINT(command[cmdNo].datastart);
			DBG_PRINT(" end:");
			DBG_PRINTLN(command[cmdNo].dataend);
			//if (type==raw) send_raw(&msg_part,buckets);
			//if (type==manchester) send_mc(&msg_part,sendclock);
			//digitalWrite(PIN_SEND, LOW); // turn off transmitter
			//digitalLow(PIN_SEND);
		}
		else if (msg_part.charAt(0) == 'C' && msg_part.charAt(1) == '=')
		{
			//sendclock = msg_part.substring(2).toInt();
			command[cmdNo].sendclock = msg_part.substring(2).toInt();
			DBG_PRINTLN("adding sendclock");
	}
#ifdef CMP_CC1101
		else if (msg_part.charAt(0) == 'F' && msg_part.charAt(1) == '=')
		{
			ccParamAnz = msg_part.length() / 2 - 1;

			if (ccParamAnz > 0 && ccParamAnz <= 5 && hasCC1101) {
				uint8_t hex;
				DBG_PRINT("write new ccreg  ");
				for (uint8_t i=0;i<ccParamAnz;i++)
				{
					ccReg[i] = cc1101::readReg(0x0d + i, 0x80);    // alte Registerwerte merken
					hex = (uint8_t)msg_part.charAt(2 + i*2);
					val = cc1101::hex2int(hex) * 16;
					hex = (uint8_t)msg_part.charAt(3 + i*2);
					val = cc1101::hex2int(hex) + val;
					cc1101::writeReg(0x0d + i, val);            // neue Registerwerte schreiben
					printHex2Dbg(val);
				}
				DBG_PRINTLN("");
			}
		}
#endif  // CMP_CC1101
}

#ifdef CMP_CC1101
	if (hasCC1101) cc1101::setTransmitMode();
#endif


	if (command[0].type == combined && command[0].repeats > 0) {
		repeats = command[0].repeats;
	}
	for (uint8_t i = 0; i < repeats; i++)
	{
		DBG_PRINT("repeat "); DBG_PRINT(i); DBG_PRINT("/"); DBG_PRINT(repeats);

		for (uint8_t c = 0; c <= cmdNo; c++)
		{
			DBG_PRINT(" cmd "); DBG_PRINT(c); DBG_PRINT("/"); DBG_PRINT(cmdNo);
			DBG_PRINT(" reps "); DBG_PRINT(command[c].repeats);

			if (command[c].type == raw) { for (uint8_t rep = 0; rep < command[c].repeats; rep++) send_raw(command[c].datastart, command[c].dataend, command[c].buckets); }
			else if (command[c].type == manchester) { for (uint8_t rep = 0; rep < command[c].repeats; rep++)send_mc(command[c].datastart, command[c].dataend, command[c].sendclock); }
			digitalLow(PIN_SEND);
			DBG_PRINT(".");

		}
		DBG_PRINTLN(" ");

		if (extraDelay) delay(1);
	}

#ifdef CMP_CC1101
	if (ccParamAnz > 0) {
		DBG_PRINT("ccreg write back ");
		for (uint8_t i = 0; i<ccParamAnz; i++)
		{
			val = ccReg[i];
			printHex2Dbg(val);
			cc1101::writeReg(0x0d + i, val);    // gemerkte Registerwerte zurueckschreiben
		}
		DBG_PRINTLN("");
	}
#endif  // CMP_CC1101
		
	MSG_PRINTLN(cmdstring); // echo
	musterDec.reset();
	enableReceive();	// enable the receiver
}





//================================= Kommandos ======================================

void HandleCommand()
{
	uint8_t reg;
	uint8_t val;
#define  cmd_Version 'V'
#define  cmd_freeRam 'R'
#define  cmd_intertechno 'i'
#define  cmd_uptime 't'
#define  cmd_changeReceiver 'X'
#define  cmd_space ' '
#define  cmd_help '?'
#define  cmd_changeFilter 'F'
#define  cmd_send 'S'
#define  cmd_ping 'P'
#define  cmd_config 'C'
#define  cmd_buffer 'B'
#define  cmd_write 'W'      // write EEPROM und write CC1101 register
#define  cmd_read  'r'      // read EEPROM
#define  cmd_patable 'x' 
#define  cmd_ccFactoryReset 'e'  // EEPROM / factory reset

 

	if (cmdstring.charAt(0) == cmd_ping) {
		getPing();
	}  // ?: Kommandos anzeigen
	else if (cmdstring.charAt(0) == cmd_help) {
		MSG_PRINT(cmd_help);	MSG_PRINT(F(" Use one of "));
		MSG_PRINT(cmd_Version); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_freeRam); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_uptime); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_changeReceiver); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_changeFilter); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_send); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_ping); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_config); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_read); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_write); MSG_PRINT(cmd_space);
		if (hasCC1101) {
			MSG_PRINT(cmd_patable); MSG_PRINT(cmd_space);
			MSG_PRINT(cmd_ccFactoryReset); MSG_PRINT(cmd_space);
		}
		MSG_PRINTLN("");
	}
	// V: Version
	else if (cmdstring.charAt(0) == cmd_Version) {
    MSG_PRINT("V " PROGVERS " SIGNALESP ");
#ifdef CMP_CC1101
    if (hasCC1101) {
      MSG_PRINT(F("cc1101"));
      switch(cc1101::chipVersion()) {
        case 0x08:  // CC1101_VERSION 0x31
        case 0x18:  // CC1101_VERSION 0xF1
          MSG_PRINT(F(" 433MHz"));
          break;
        case 0x04:  // CC1101_VERSION 0x31
        case 0x14:  // CC1101_VERSION 0xF1
          MSG_PRINT(F(" 868MHz"));
          break;
        default:
          MSG_PRINT(" chip unknown 0x" + String(cc1101::chipVersion(), HEX));
          break;
      }
    }
#endif
		MSG_PRINTLN(" - compiled at " __DATE__ " " __TIME__);
	}
	// R: FreeMemory
	else if (cmdstring.charAt(0) == cmd_freeRam) {

		MSG_PRINTLN(freeRam());
	}
	// i: Intertechno
	else if (cmdstring.charAt(0) == cmd_intertechno) {
		if (musterDec.getState() != searching)
		{
			command_available = true;
		}
	}
	else if (cmdstring.charAt(0) == cmd_send) {
		if (musterDec.getState() != searching)
		{
			command_available = true;
		}
		else {
			send_cmd(); // Part of Send
		}
	}
	// t: Uptime
	else if (cmdstring.charAt(0) == cmd_uptime) {
		MSG_PRINTLN(getUptime());
	}
	// XQ disable receiver
	else if (cmdstring.charAt(0) == cmd_changeReceiver) {
		changeReciver();
	}
	else if (cmdstring.charAt(0) == cmd_changeFilter) {
	}
	else if (cmdstring.charAt(0) == cmd_config) {
		if (cmdstring.charAt(1) == 'G') {
			getConfig();
		}
		else if (cmdstring.charAt(1) == 'E' || cmdstring.charAt(1) == 'D') {  //Todo:  E und D sind auch hexadezimal, werden hier aber abgefangen
			configCMD();
      EEPROM.commit();
		}
		else if (cmdstring.charAt(1) == 'S') {
			configSET();
		}
#ifdef CMP_CC1101
		else if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2)) && hasCC1101) {
			reg = cmdstringPos2int(1);
			cc1101::readCCreg(reg);
		}
#endif
		else {
			MSG_PRINTLN(F("Unsupported command"));
		}
	}
#ifdef CMP_CC1101
	else if (cmdstring.charAt(0) == cmd_write) {            // write EEPROM und CC11001 register
		if (cmdstring.charAt(1) == 'S' && cmdstring.charAt(2) == '3' && hasCC1101) {       // WS<reg>  Command Strobes
			cc1101::commandStrobes();
		}
		else if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2)) && isHexadecimalDigit(cmdstring.charAt(3)) && isHexadecimalDigit(cmdstring.charAt(4))) {
			reg = cmdstringPos2int(1);
			val = cmdstringPos2int(3);
			EEPROM.write(reg+1, val); // scheinbar hat sich hier etwas um 1 Byte verschoben
			EEPROM.commit();
			if (hasCC1101) {
				cc1101::writeCCreg(reg, val);
			}
		}
		else {
			MSG_PRINTLN(F("Unsupported command"));
		}
	}
	// R<adr>  read EEPROM
	else if (cmdstring.charAt(0) == cmd_read && isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2))) {             // R<adr>  read EEPROM
		reg = cmdstringPos2int(1);
		MSG_PRINT(F("EEPROM "));
		printHex2(reg);
		if (cmdstring.charAt(3) == 'n') {
			MSG_PRINT(F(" :"));
			for (uint8_t i = 0; i < 16; i++) {
				MSG_PRINT(" ");
				printHex2(EEPROM.read(reg + i));
			}
		}
		else {
			MSG_PRINT(F(" = "));
			printHex2(EEPROM.read(reg));
		}
		MSG_PRINTLN("");
	}
	else if (cmdstring.charAt(0) == cmd_patable && isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2)) && hasCC1101) {
		val = cmdstringPos2int(1);
		cc1101::writeCCpatable(val);
		MSG_PRINT(F("Write "));
		printHex2(val);
		MSG_PRINTLN(F(" to PATABLE done"));
	}
	else if (cmdstring.charAt(0) == cmd_ccFactoryReset && hasCC1101) {
		cc1101::ccFactoryReset();
		EEPROM.commit();
		cc1101::CCinit();
	}
#endif
	else {
		MSG_PRINT(F("Unsupported command"));
		MSG_PRINTLN(" -> 0x" + String(cmdstring.charAt(0), HEX) + " " + cmdstring);
	}
}


void getConfig()
{
	MSG_PRINT(F("MS="));
	MSG_PRINT(musterDec.MSenabled, DEC);
	MSG_PRINT(F(";MU="));
	MSG_PRINT(musterDec.MUenabled, DEC);
	MSG_PRINT(F(";MC="));
	MSG_PRINTLN(musterDec.MCenabled, DEC);
}

/*
void enDisPrint(bool enDis)
{
	if (enDis) {
		MSG_PRINT(F("enable"));
	}
	else {
		MSG_PRINT(F("disable"));
	}
}
*/

void configCMD()
{
	if (cmdstring.charAt(1) == 'G') {  // Get, no change to configuration
		getConfig();
		return;
	}

	bool *bptr;

	if (cmdstring.charAt(2) == 'S') {  	  //MS
		bptr = &musterDec.MSenabled;;
	}
	else if (cmdstring.charAt(2) == 'U') {  //MU
		bptr = &musterDec.MUenabled;;
	}
	else if (cmdstring.charAt(2) == 'C') {  //MC
		bptr = &musterDec.MCenabled;;
	}

	if (cmdstring.charAt(1) == 'E') {   // Enable
		*bptr = true;
	}
	else if (cmdstring.charAt(1) == 'D') {  // Disable
		*bptr = false;
#ifdef CMP_CC1101
	} else if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2)) && hasCC1101) {
    uint8_t reg = cmdstringPos2int(1);
    cc1101::readCCreg(reg);
#endif
  }	else {
		return;
	}
	storeFunctions(musterDec.MSenabled, musterDec.MUenabled, musterDec.MCenabled);
}

inline void configSET()
{
	//MSG_PRINT(cmdstring.substring(2, 8));
	if (cmdstring.substring(2, 8) == "mcmbl=")    // mc min bit len
	{
		musterDec.mcMinBitLen = cmdstring.substring(8).toInt();
		MSG_PRINT(musterDec.mcMinBitLen); MSG_PRINT(" bits set");
	}
}


inline void ethernetEvent()
{
	//check if there are any new clients
	if (Server.hasClient()) {
		if (!serverClient || !serverClient.connected()) {
			if (serverClient) serverClient.stop();
			serverClient = Server.available();
			DBG_PRINT("New client: ");
			DBG_PRINTLN(serverClient.remoteIP());
			return;
		}
		//no free/disconnected spot so reject
//		WiFiClient newClient = Server.available();
//		newClient.stop();
	}
	//yield();
}

void serialEvent()
{
	while (MSG_PRINTER.available())
	{
		char inChar = (char)MSG_PRINTER.read();
		switch (inChar)
		{
		case '\n':
		case '\r':
		case '\0':
		case '#':
			command_available = true;
			break;
		default:
			cmdstring += inChar;
		}
		yield();
	}
}


int freeRam() {

	return system_get_free_heap_size();
	

}

unsigned long getUptime()
{
	unsigned long now = millis();
	static uint16_t times_rolled = 0;
	static unsigned long last = 0;
	// If this run is less than the last the counter rolled
	unsigned long seconds = now / 1000;
	if (now < last) {
		times_rolled++;
		seconds += ((long(4294967295) / 1000)*times_rolled);
	}
	last = now;
	return seconds;
}

void getPing()
{
	MSG_PRINTLN("OK");
	yield();
}

void changeReciver() {
	if (cmdstring.charAt(1) == 'Q')
	{
		disableReceive();
	}
	if (cmdstring.charAt(1) == 'E')
	{
		enableReceive();
	}
}






//================================= EEProm commands ======================================



void storeFunctions(const int8_t ms, int8_t mu, int8_t mc)
{
	mu = mu << 1;
	mc = mc << 2;
	int8_t dat = ms | mu | mc;
	EEPROM.write(addr_features, dat);
}

void getFunctions(bool *ms, bool *mu, bool *mc)
{
	int8_t dat = EEPROM.read(addr_features);
	*ms = bool(dat &(1 << 0));
	*mu = bool(dat &(1 << 1));
	*mc = bool(dat &(1 << 2));
}

void dumpEEPROM() {
  Serial.println("\ndump EEPROM:");
  for (uint8_t i=0; i<56; i++) {
    String temp=String(EEPROM.read(i), HEX);
    Serial.print((temp.length() == 1 ? "0" : "") + temp + " ");
    if ((i & 0x0F) == 0x0F)
      Serial.println("");
  }
  Serial.println("");
}

void initEEPROM() {
	EEPROM.begin(512); //Max bytes of eeprom to use

	if (EEPROM.read(EE_MAGIC_OFFSET) == VERSION_1 && EEPROM.read(EE_MAGIC_OFFSET + 1) == VERSION_2) {
		DBG_PRINTLN("Reading values fom eeprom");
	} else {
    DBG_PRINTLN("Init eeprom to defaults after flash");
    EEPROM.write(EE_MAGIC_OFFSET, VERSION_1);
    EEPROM.write(EE_MAGIC_OFFSET + 1, VERSION_2);
		storeFunctions(1, 1, 1);    // Init EEPROM with all flags enabled
									 //hier fehlt evtl ein getFunctions()
#ifdef CMP_CC1101
		cc1101::ccFactoryReset();
#endif

    EEPROM.commit();
	}

#ifdef DEBUG
  dumpEEPROM();
#endif

	getFunctions(&musterDec.MSenabled, &musterDec.MUenabled, &musterDec.MCenabled);
}

#ifdef CMP_CC1101 
uint8_t cmdstringPos2int(uint8_t pos) {
  uint8_t val;
  uint8_t hex;
  
   hex = (uint8_t)cmdstring.charAt(pos);
   val = cc1101::hex2int(hex) * 16;
   hex = (uint8_t)cmdstring.charAt(pos+1);
   val = cc1101::hex2int(hex) + val;
   return val;
}

void printHex2(const byte hex) {   // Todo: printf oder scanf nutzen
  if (hex < 16) {
    MSG_PRINT("0");
  }
  MSG_PRINT(hex, HEX);
}
void printHex2Dbg(const byte hex) {   // Todo: printf oder scanf nutzen
  if (hex < 16) {
    DBG_PRINT("0");
  }
  DBG_PRINT(hex, HEX);
}
#endif


String uptime() {
  String result = "";

  unsigned long uptime = (millis() / 1000);

  uint8_t uptimeDays = uptime / 86400;
  if (uptimeDays > 0)
    result += String(uptimeDays) + "d, ";
  uptime %= 86400;
  uint8_t hours = uptime / 3600;
  result += String(hours < 10 ? String("0") + hours : hours) + ":";
  uptime %= 3600;
  uint8_t minutes = uptime / 60;
  result += String(minutes < 10 ? String("0") + minutes : minutes) + ".";
  uptime %= 60;
  result += String(uptime < 10 ? String("0") + uptime : uptime);

  return result;
}

