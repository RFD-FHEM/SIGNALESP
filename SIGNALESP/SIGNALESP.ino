

#define PROGNAME               "RF_RECEIVER"
#define PROGVERS               "3.3.0"


#define PIN_RECEIVE            2
#define PIN_LED                16
#define PIN_SEND               0
#define BAUDRATE               115200
#define FIFO_LENGTH			   100
#define DEBUG				   1

#include "configwifi.h"

#if WIFI_Enable 1	
		#define ETHERNET_PRINT
		#include <ESP8266WiFi.h>

		WiFiServer server(23);
		WiFiClient serverClients[1];

#endif

#define MAX_SRV_CLIENTS			1



#include <output.h>
#include <bitstore.h>  // Die wird aus irgend einem Grund zum Compilieren benoetigt.
#include <SimpleFIFO.h>

SimpleFIFO<int, FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
#include <signalDecoder.h>
SignalDetectorClass musterDec;


#include <EEPROM.h>


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




// EEProm Addresscommands
#define addr_init 0
#define addr_features 1



//void handleInterrupt();
void enableReceive();
void disableReceive();
void serialEvent();
void cronjob();
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




void setup() {
	ESP.wdtEnable(1500);
	Serial.begin(BAUDRATE);
#ifdef DEBUG
	Serial.println("Using sFIFO");
#endif
	//pinMode(PIN_RECEIVE, INPUT);
	//pinMode(PIN_SEND, OUTPUT);
	//pinMode(PIN_LED, OUTPUT);
	os_timer_setfn(&cronTimer, cronjob, NULL);
	os_timer_arm(&cronTimer, 31, true);
	
	musterDec.MSenabled = musterDec.MUenabled = musterDec.MCenabled = true;
	/*
	if (EEPROM.read(addr_init) == 0xB)
	{
	#ifdef DEBUG
	Serial.println("Reading values fom eeprom");
	#endif
	getFunctions(&musterDec.MSenabled, &musterDec.MUenabled, &musterDec.MCenabled);
	}
	else {
	EEPROM.write(addr_init, 0xB);
	storeFunctions(1, 1, 1);    // Init EEPROM with all flags enabled
	#ifdef DEBUG
	Serial.println("Init eeprom to defaults after flash");
	#endif
	}*/
	//WiFi.begin(ssid, password);

	enableReceive();
	cmdstring.reserve(20);
}

void cronjob(void *pArg) {

	digitalWrite(PIN_LED, blinkLED);
	blinkLED = false;

}

void loop() {
	static int aktVal = 0;
	bool state;
	serialEvent();
	if (command_available) {
		command_available = false;
		HandleCommand();
		if (!command_available) { cmdstring = ""; }
		blinkLED = true;
	}
	yield();

	while (FiFo.count()>0) { //Puffer auslesen und an Dekoder uebergeben

		aktVal = FiFo.dequeue();
		state = musterDec.decode(&aktVal);
		if (state) blinkLED = true; //LED blinken, wenn Meldung dekodiert
		yield();
	}
}



//========================= Pulseauswertung ================================================
void handleInterrupt() {
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
	attachInterrupt(PIN_RECEIVE, handleInterrupt, CHANGE);
}

void disableReceive() {
	detachInterrupt(PIN_RECEIVE);
}




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
			;
		}
		isLow ? digitalLow(PIN_SEND) : digitalHigh(PIN_SEND);
		stoptime += dur;
	}
	while (stoptime > micros()) {
		;
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
	int16_t sendclock;
	uint8_t type;
	uint8_t datastart;
	uint16_t dataend;
	int16_t buckets[6];
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

	disableReceive();

	uint8_t cmdNo = 255;


	while (split_cmdpart(&start_pos, &msg_part))
	{
		//MSG_PRINTLN(msg_part);
		if (msg_part.charAt(0) == 'S')
		{
			if (msg_part.charAt(1) == 'C')  // send combined informatio flag
			{
				//type=combined;
				//cmdNo=255;
				extraDelay = false;
			}
			else if (msg_part.charAt(1) == 'M') // send manchester
			{
				//type=manchester;
				cmdNo++;
				command[cmdNo].type = manchester;
				//MSG_PRINTLN("Adding manchester");
			}
			else if (msg_part.charAt(1) == 'R') // send raw
			{
				//type=raw;
				cmdNo++;
				command[cmdNo].type = raw;
				//MSG_PRINTLN("Adding raw");
				extraDelay = false;

			}
		}
		else if (msg_part.charAt(0) == 'P' && msg_part.charAt(2) == '=') // Do some basic detection if data matches what we expect
		{
			counter = msg_part.substring(1, 2).toInt(); // extract the pattern number
														//buckets[counter]=  msg_part.substring(3).toInt();
			command[cmdNo].buckets[counter] = msg_part.substring(3).toInt();
			//MSG_PRINTLN("Adding bucket");

		}
		else if (msg_part.charAt(0) == 'R' && msg_part.charAt(1) == '=') {
			repeats = msg_part.substring(2).toInt();
			//MSG_PRINTLN("Adding repeats");

		}
		else if (msg_part.charAt(0) == 'D') {
			command[cmdNo].datastart = start_pos - msg_part.length() + 1;
			command[cmdNo].dataend = start_pos - 2;
			//MSG_PRINT("locating data start:");
			// MSG_PRINT(command[cmdNo].datastart);
			//MSG_PRINT(" end:");
			//MSG_PRINTLN(command[cmdNo].dataend);
			//if (type==raw) send_raw(&msg_part,buckets);
			//if (type==manchester) send_mc(&msg_part,sendclock);
			//digitalWrite(PIN_SEND, LOW); // turn off transmitter
			//digitalLow(PIN_SEND);
		}
		else if (msg_part.charAt(0) == 'C' && msg_part.charAt(1) == '=')
		{
			//sendclock = msg_part.substring(2).toInt();
			command[cmdNo].sendclock = msg_part.substring(2).toInt();
			//MSG_PRINTLN("adding sendclock");
		}
	}

	for (uint8_t i = 0; i<repeats; i++)
	{
		for (uint8_t c = 0; c <= cmdNo; c++)
		{
			if (command[c].type == raw) send_raw(command[c].datastart, command[c].dataend, command[c].buckets);
			if (command[c].type == manchester) send_mc(command[c].datastart, command[c].dataend, command[c].sendclock);
			digitalLow(PIN_SEND);
		}
		if (extraDelay) delay(1);
	}

	enableReceive();	// enable the receiver
	MSG_PRINTLN(cmdstring); // echo

}





//================================= Kommandos ======================================

void HandleCommand()
{

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
#define  cmd_getConfig 'G' //decrepated


	if (cmdstring.charAt(0) == cmd_ping) {
		getPing();
	}  // ?: Kommandos anzeigen
	else if (cmdstring.charAt(0) == cmd_help) {
		MSG_PRINT(cmd_help);	MSG_PRINT(F(" Use one of "));
		MSG_PRINT(cmd_Version); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_intertechno); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_freeRam); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_uptime); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_changeReceiver); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_changeFilter); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_send); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_ping); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_config); MSG_PRINT(cmd_space);
		MSG_PRINT(cmd_getConfig); MSG_PRINT(cmd_space);  //decrepated

		MSG_PRINTLN("");
	}
	// V: Version
	else if (cmdstring.charAt(0) == cmd_Version) {
		MSG_PRINTLN("V " PROGVERS " SIGNALESP - compiled at " __DATE__ " " __TIME__);
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
		configCMD();
	}
	// get config
	else if (cmdstring.charAt(0) == cmd_getConfig) {
		getConfig();
	}
	else {
		MSG_PRINTLN(F("Unsupported command"));
	}
}


void getConfig()
{
	MSG_PRINT(F("MS="));
	//enDisPrint(musterDec.MSenabled);
	MSG_PRINT(musterDec.MSenabled, DEC);
	MSG_PRINT(F(";MU="));
	//enDisPrint(musterDec.MUenabled);
	MSG_PRINT(musterDec.MUenabled, DEC);
	MSG_PRINT(F(";MC="));
	//enDisPrint(musterDec.MCenabled);
	MSG_PRINTLN(musterDec.MCenabled, DEC);
}


void enDisPrint(bool enDis)
{
	if (enDis) {
		MSG_PRINT(F("enable"));
	}
	else {
		MSG_PRINT(F("disable"));
	}
}


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
	}
	else {
		return;
	}
	storeFunctions(musterDec.MSenabled, musterDec.MUenabled, musterDec.MCenabled);
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

	// Todo: Get code for esp to receive free RAM
	return 0;

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

