/*
*   Pattern Decoder Library V3
*   Library to decode radio signals based on patternd detection
*   2014-2015  N.Butzek, S.Butzek
*   2015  S.Butzek
*	2016  S.Butzek

*   This library contains classes to perform decoding of digital signals
*   typical for home automation. The focus for the moment is on different sensors
*   like weather sensors (temperature, humidity Logilink, TCM, Oregon Scientific, ...),
*   remote controlled power switches (Intertechno, TCM, ARCtech, ...) which use
*   encoder chips like PT2262 and EV1527-type and manchester encoder to send
*   information in the 433MHz or 868 Mhz Band.
*
*   The classes in this library follow the approach to detect a recurring pattern in the
*   recived signal. For Manchester there is a class which decodes the signal.
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include "signalDecoder.h"


void SignalDetectorClass::bufferMove(const uint8_t start)
{
	static uint8_t len_single_entry = sizeof(*message);

	if (start > messageLen-1) return;

	messageLen = messageLen - start; // Berechnung der neuen Nachrichtenl�nge nach dem L�schen
	memmove(message, message + start, len_single_entry*messageLen);

}


inline void SignalDetectorClass::addData(const uint8_t value)
{
	message[messageLen] = value;
	messageLen++;
}

inline void SignalDetectorClass::addPattern()
{
	pattern[pattern_pos] = *first;						//Store pulse in pattern array
	pattern_pos++;
}

inline void SignalDetectorClass::updPattern( const uint8_t ppos)
{
	pattern[ppos] = (long(pattern[ppos]) + *first) / 2; // Moving average
}


inline void SignalDetectorClass::doDetect()
{
		 
		bool valid=true;
		valid = (messageLen==0 || (*first ^ *last) < 0); // true if a and b have opposite signs
		valid &=  (messageLen == maxMsgSize) ? false : true;


//		if (messageLen == 0) pattern_pos = patternLen = 0;
		//if (messageLen == 0) valid = true;


		if (!valid)
		{
			// Try output
			
			m_overflow = (messageLen == maxMsgSize) ? true : false;
			processMessage();
			
		}	else if (messageLen == minMessageLen) {
			state = detecting;  // Set state to detecting, because we have more than minMessageLen data gathered, so this is no noise
		}

		int8_t fidx = findpatt(*first);
		if (fidx >= 0) {
			// Upd pattern

			updPattern(fidx);
		}
		else { 			
			// Add pattern

			for (int16_t i = messageLen - 1; (patternLen == maxNumPattern) && (i >= 0); --i)
			{
				if (message[i] == pattern_pos) // Finde den letzten Verweis im Array auf den Index der gleich �berschrieben wird
				{
					i++; // i um eins erh�hen, damit zuk�nftigen Berechnungen darauf aufbauen k�nnen
					bufferMove(i);
					break;
				}
			}
			fidx = pattern_pos;
			addPattern();

			if (pattern_pos == maxNumPattern)
			{
				pattern_pos = 0;  // Wenn der Positions Index am Ende angelegt ist, gehts wieder bei 0 los und wir �berschreiben alte pattern
				patternLen = maxNumPattern;
				mcDetected = false;  // When changing a pattern, we need to redetect a manchester signal and we are not in a buffer full mode scenario

			}
			if (pattern_pos > patternLen) patternLen = pattern_pos;

		}
		
		// Add data to buffer
		addData(fidx);


#if DEBUGDETECT>3
		Serial.print("Pulse: "); Serial.print(*first); Serial.print(", "); Serial.print(*last);
		Serial.print(", TOL: "); Serial.print(tol); Serial.print(", Found: "); Serial.print(fidx);
		Serial.print(", Vld: "); Serial.print(valid);
		Serial.print(", pattPos: "); Serial.print(pattern_pos);
		Serial.print(", mLen: "); Serial.println(messageLen);
#endif
		return;

		bool add_new_pattern = true;

		if (0 <= fidx) {

			//gefunden
			if (messageLen > 0 && message[messageLen - 1] == fidx) {  // Der Fall darf eigentlich nicht vorkommen da hier Valid = 0 sein muss
				add_new_pattern = true;
				valid = false;
			} else {
				add_new_pattern = false;
				//addData(fidx);
				updPattern(fidx);
			}
		}
		else {
			// Pr�fen ob wir noch Muster in den Puffer aufnehmen k�nnen oder ob wir Muster �berschreiben w�rden
			calcHisto();
			if (patternLen == maxNumPattern && histo[pattern_pos] > 1)
			{
				valid = false;
				add_new_pattern = true;
			}
		}

		if (!valid) {
			//Serial.println("not valid, processing");

			//success = true;
			processMessage();
			// reset();  // GGF hier nicht ausf�hren.
			pattern_pos = 0;
			//doDetectwoSync(); //Sichert den aktuellen Puls nach dem Reset, da wir ihn ggf. noch ben�tigen
		
			//return;
		}
		/*else if (!valid) {
			reset();
			success = false;
			pattern_pos = 0;
		}*/
		else if (valid && messageLen >= minMessageLen) {
			state = detecting;  // Set state to detecting, because we have more than minMessageLen data gathered, so this is no noise
		}
		/*else {
		if (messageLen>=minMessageLen){
		// Annahme, wir haben eine Nachricht empfangen, jetzt kommt rauschen, welches nicht zum Muster passt
		//printOut();
		//processMessage();
		//reset();pattern_pos=0;
		}
		*/
		if (add_new_pattern)
		{
			// L�scht alle Eintr�ge in dem Nachrichten Array die durch das hinzug�gen eines neuen Pattern �berschrieben werden
			// Array wird sozusagen nach links geschoben
			
			for (uint8_t  i = messageLen - 1; (i >= 0); --i)
			{
				if (message[i] == pattern_pos) // Finde den letzten Verweis im Array auf den Index der gleich �berschrieben wird
				{
					i++; // i um eins erh�hen, damit zuk�nftigen Berechnungen darauf aufbauen k�nnen
					bufferMove(i);
					messageLen++;  //Move messagelen pointer one forward to avoid overwrite
					break;
				}
			}
			pattern[pattern_pos] = *first;						//Store pulse in pattern array
			message[messageLen] = pattern_pos;
#if DEBUGDETECT>3
			Serial.print(F(", pattPos: ")); Serial.print(pattern_pos);
#endif // DEBUGDETECT
			//*(message+messageLen) = patternLen; 					//Index des letzten Elements in die Nachricht schreiben

			messageLen++;
			pattern_pos++;

			//printOut();
			if (pattern_pos == maxNumPattern)
			{
				pattern_pos = 0;  // Wenn der Positions Index am Ende angelegt ist, gehts wieder bei 0 los und wir �berschreiben alte pattern
				patternLen = maxNumPattern;
			}

			mcDetected = false;  // When changing a pattern, we need to redetect a manchester signal and we are not in a buffer full mode scenario

			/*
			if (pattern_pos==maxNumPattern)
			{
			pattern_pos=0;  // Wenn der Positions Index am Ende angelegt ist, gehts wieder bei 0 los und wir �berschreiben alte pattern
			patternLen=maxNumPattern;
			} else {

			patternLen++;
			}
			*/
			/*
			DEBUG_BEGIN(2)
			printOut();
			DEBUG_END
			*/
		}
#if DEBUGDETECT>3
		Serial.println();
#endif // DEBUGDETECT



}

bool SignalDetectorClass::decode(const int * pulse)
{
	success = false;

	//int temp;
	//*first = *last;
	//*last = *pulse;
	if (messageLen > 0)
		last = &pattern[message[messageLen - 1]];
	*first = *pulse;
	
	doDetect();
	return success;
}


void SignalDetectorClass::compress_pattern()
{

	calcHisto();
	for (uint8_t idx = 0; idx<patternLen; idx++)
	{
		for (uint8_t idx2 = idx + 1; idx2<patternLen; idx2++)
		{
			const int16_t tol = int((abs(pattern[idx2])*tolFact) + (abs(pattern[idx2])*tolFact) / 2);
			if (inTol(pattern[idx2], pattern[idx], tol))  // Pattern are very equal, so we can combine them
			{
				// Change val -> ref_val in message array
				for (uint8_t i = 0; i<messageLen; i++)
				{
					if (message[i] == idx2)
					{
						message[i] = idx;
					}
				}

#if DEBUGDETECT>2
				Serial.print("compr: "); Serial.print(idx2); Serial.print("->"); Serial.print(idx); Serial.print(";");
				Serial.print(histo[idx2]); Serial.print("*"); Serial.print(pattern[idx2]);
				Serial.print("->");
				Serial.print(histo[idx]); Serial.print("*"); Serial.print(pattern[idx]);
#endif // DEBUGDETECT


				int  sum = histo[idx] + histo[idx2];
				if (sum == 0)
					pattern[idx] = (long(pattern[idx]) * histo[idx] / sum) + (pattern[idx2] * histo[idx2] / sum);
				else
					pattern[idx] = (long(pattern[idx]) + pattern[idx2]) / 2;
				//pattern[idx][0] = (pattern[idx][0]*float(histo[idx]/ sum))+(pattern[idx2][0]*float(histo[idx2]/ sum)); // Store the average of both pattern, may better to calculate the number of stored pattern in message
				//pattern[idx][0] = (pattern[idx][0]+pattern[idx2][0])/2;
				pattern[idx2] = 0;

#if DEBUGDETECT>2
				Serial.print(" idx:"); Serial.print(pattern[idx]);
				Serial.print(" idx2:"); Serial.print(pattern[idx2]);
				Serial.println(";");
#endif // DEBUGDETECT

			}
		}
	}
}

void SignalDetectorClass::processMessage()
{
	
	if (mcDetected == true || messageLen >= minMessageLen) {
		success = false;
#if DEBUGDETECT >= 1
		Serial.println("Message received:");
#endif
		compress_pattern();
		calcHisto();
		getClock();
		if (state == clockfound) getSync();

#if DEBUGDETECT >= 1
		printOut();
#endif

		if (MSenabled && state == syncfound && messageLen >= minMessageLen)// Messages mit clock / Sync Verh�ltnis pr�fen
		{
#if DEBUGDECODE >0
			Serial.print(" MS check: ");

			//printOut();
#endif	

			// Setup of some protocol identifiers, should be retrieved via fhem in future

			mend = mstart + 2;   // GGf. kann man die Mindestl�nge von x Signalen vorspringen
			bool m_endfound = false;

			//uint8_t repeat;
			while (mend < messageLen - 1)
			{
				if (message[mend + 1] == sync && message[mend] == clock) {
					mend -= 1;					// Previus signal is last from message
					m_endfound = true;
					break;
				}
				mend += 2;
			}
			if (mend > messageLen) mend = messageLen;  // Reduce mend if we are behind messageLen
													   //if (!m_endfound) mend=messageLen;  // Reduce mend if we are behind messageLen

			calcHisto(mstart, mend);	// Recalc histogram due to shortened message


#if DEBUGDECODE > 1
			Serial.print("Index: ");
			Serial.print(" MStart: "); Serial.print(mstart);
			Serial.print(" SYNC: "); Serial.print(sync);
			Serial.print(", CP: "); Serial.print(clock);
			Serial.print(" - MEFound: "); Serial.println(m_endfound);
			Serial.print(" - MEnd: "); Serial.println(mend);
#endif // DEBUGDECODE
			if ((m_endfound && (mend - mstart) >= minMessageLen) || (!m_endfound && messageLen < (maxMsgSize)))//(!m_endfound && messageLen  >= minMessageLen))	// Check if message Length is long enough
			{
#ifdef DEBUGDECODE
				Serial.println("Filter Match: ");;
#endif


				preamble = "";
				postamble = "";

				/*				Output raw message Data				*/
				preamble.concat(MSG_START);
				//preamble.concat('\n');
				preamble.concat("MS");   // Message Index
										 //preamble.concat(int(pattern[sync][0]/(float)pattern[clock][0]));
				preamble.concat(SERIAL_DELIMITER);  // Message Index
				for (uint8_t idx = 0; idx < patternLen; idx++)
				{
					if (histo[idx] == 0) continue;
					preamble.concat('P'); preamble.concat(idx); preamble.concat("="); preamble.concat(pattern[idx]); preamble.concat(SERIAL_DELIMITER);  // Patternidx=Value
				}
				preamble.concat("D=");

				postamble.concat(SERIAL_DELIMITER);
				postamble.concat("CP="); postamble.concat(clock); postamble.concat(SERIAL_DELIMITER);    // ClockPulse
				postamble.concat("SP="); postamble.concat(sync); postamble.concat(SERIAL_DELIMITER);     // SyncPu�se
				if (m_overflow) {
					postamble.concat("O");
					postamble.concat(SERIAL_DELIMITER);
				}
				postamble.concat(MSG_END);
				postamble.concat('\n');

				printMsgRaw(mstart, mend, &preamble, &postamble);
				success = true;

#ifdef mp_crc
				const int8_t crco = printMsgRaw(mstart, mend, &preamble, &postamble);

				if ((mend < messageLen - minMessageLen) && (message[mend + 1] == message[mend - mstart + mend + 1])) {
					mstart = mend + 1;
					byte crcs = 0x00;
#ifndef ARDUSIM
					for (uint8_t i = mstart + 1; i <= mend - mstart + mend; i++)
					{
						crcs = _crc_ibutton_update(crcs, message[i]);
					}
#endif
					if (crcs == crco)
					{
						// repeat found
					}
					//processMessage(); // Todo: needs to be optimized
				}
#endif


			}
			else if (m_endfound == false && mstart > 1 && mend + 1 >= maxMsgSize) // Start found, but no end. We remove everything bevore start and hope to find the end later
			{
				//Serial.print("copy");
#ifdef DEBUGDECODE
				Serial.print(" move msg ");;
#endif
				bufferMove(mstart);
				m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
			}
			else {
#ifdef DEBUGDECODE
				Serial.println(" Buffer overflow, flushing message array");
#endif
				//Serial.print(MSG_START);
				//Serial.print("Buffer overflow while processing signal");
				//Serial.print(MSG_END);
				reset(); // Our Messagebuffer is not big enough, no chance to get complete Message

			}
		}
		if (success == false && (MUenabled || MCenabled)) {

#if DEBUGDECODE >0
			Serial.print(" MU/MC check: ");

			//printOut();
#endif	
// Message has a clock puls, but no sync. Try to decode this

			preamble = "";
			postamble = "";


			//String preamble;

			preamble.concat(MSG_START);
			if (MCenabled)
			{
				static ManchesterpatternDecoder mcdecoder(this);			// Init Manchester Decoder class

				if (mcDetected == false)
				{
					mcdecoder.reset();
					mcdecoder.setMinBitLen(17);							// Todo: allow modification via command
				}
#if DEBUGDETECT>3
				Serial.print("vcnt: "); Serial.print(mcdecoder.ManchesterBits.valcount);
#endif;

				if ((mcDetected || mcdecoder.isManchester()) && mcdecoder.doDecode())	// Check if valid manchester pattern and try to decode
				{
#if DEBUGDECODE > 1
					Serial.print(" MC found: ");
#endif // DEBUGDECODE

					String mcbitmsg;
					//Serial.println("MC");
					mcbitmsg = "D=";
					mcdecoder.getMessageHexStr(&mcbitmsg);
					//Serial.println("f");


					preamble.concat("MC");
					preamble.concat(SERIAL_DELIMITER);
					mcdecoder.getMessagePulseStr(&preamble);

					postamble.concat(SERIAL_DELIMITER);
					mcdecoder.getMessageClockStr(&postamble);
					mcdecoder.getMessageLenStr(&postamble);


					postamble.concat(MSG_END);
					postamble.concat('\n');

					//messageLen=messageLen-mend; // Berechnung der neuen Nachrichtenl�nge nach dem L�schen
					//memmove(message,message+mend,sizeof(*message)*(messageLen+1));
					//m_truncated=true;  // Flag that we truncated the message array and want to receiver some more data

					//preamble = String(MSG_START)+String("MC")+String(SERIAL_DELIMITER)+preamble;
					//printMsgRaw(0,messageLen,&preamble,&postamble);

					//preamble.concat("MC"); ; preamble.concat(SERIAL_DELIMITER);  // Message Index

					// Output Manchester Bits
#ifdef DEBUGDECODE
					Serial.println(" ");
#endif

					printMsgStr(&preamble, &mcbitmsg, &postamble);
					mcDetected = false;
					success = true;

#if DEBUGDECODE == 1
					preamble = "MC";
					preamble.concat(SERIAL_DELIMITER);

					for (uint8_t idx = 0; idx < patternLen; idx++)
					{
						if (histo[idx] == 0) continue;

						preamble.concat("P"); preamble.concat(idx); preamble.concat("="); preamble.concat(pattern[idx]); preamble.concat(SERIAL_DELIMITER);  // Patternidx=Value
					}
					preamble.concat("D=");

					//String postamble;
					postamble = String(SERIAL_DELIMITER);
					postamble.concat("CP="); postamble.concat(clock); postamble.concat(SERIAL_DELIMITER);    // ClockPulse, (not valid for manchester)
					if (m_overflow) {
						postamble.concat("O");
						postamble.concat(SERIAL_DELIMITER);
					}

					postamble.concat(MSG_END);
					postamble.concat('\n');

					printMsgRaw(0, messageLen, &preamble, &postamble);
#endif

				}
				else if (mcDetected == true && m_truncated == true) {
					success = true;   // Prevents MU Processing
				}

			}
			if (MUenabled && state == clockfound && success == false && messageLen >= minMessageLen) {

#if DEBUGDECODE > 1
				Serial.print(" MU found: ");
#endif // DEBUGDECODE

				//preamble = String(MSG_START)+String("MU")+String(SERIAL_DELIMITER)+preamble;

				preamble.concat("MU");
				preamble.concat(SERIAL_DELIMITER);

				for (uint8_t idx = 0; idx < patternLen; idx++)
				{
					if (histo[idx] == 0) continue;

					preamble.concat("P"); preamble.concat(idx); preamble.concat("="); preamble.concat(pattern[idx]); preamble.concat(SERIAL_DELIMITER);  // Patternidx=Value
				}
				preamble.concat("D=");

				//String postamble;
				postamble.concat(SERIAL_DELIMITER);
				postamble.concat("CP="); postamble.concat(clock); postamble.concat(SERIAL_DELIMITER);    // ClockPulse, (not valid for manchester)
				if (m_overflow) {
					postamble.concat("O");
					postamble.concat(SERIAL_DELIMITER);
				}
				postamble.concat(MSG_END);
				postamble.concat('\n');

				printMsgRaw(0, messageLen - 1, &preamble, &postamble);
				m_truncated = false;
				success = true;
			}



		}
		
		if (success == false) 
		{
#if DEBUGDETECT >= 1
			Serial.println("nothing to to");
#endif
		}
	}
	if (!m_truncated)
	{
		reset();
	}
	//Serial.println("process finished");
}






void SignalDetectorClass::reset()
{
	messageLen = 0;
	patternLen = 0;
	pattern_pos = 0;
	bitcnt = 0;
	state = searching;
	clock = sync = -1;
	for (uint8_t i = 0; i<maxNumPattern; ++i)
	  histo[i] = pattern[i] = 0;
	success = false;
	tol = 150; //
	tolFact = 0.2;
	mstart = 0;
	m_truncated = false;
	m_overflow = false;
	mcDetected = false;
	//Serial.println("reset");
	mend = 0;
}

const status SignalDetectorClass::getState()
{
	return status();
}


const bool SignalDetectorClass::inTol(const int val, const int set, const int tolerance)
{
	
	// tolerance = tolerance == 0 ? tol : tolerance;
	//return (abs(val - set) <= tolerance == 0 ? tol: tolerance);
	return (abs(val - set) <= tolerance);
}

void SignalDetectorClass::printOut()
{
	Serial.println();
	Serial.print("Sync: "); Serial.print(pattern[sync]);
	Serial.print(" -> SyncFact: "); Serial.print(pattern[sync] / (float)pattern[clock]);
	Serial.print(", Clock: "); Serial.print(pattern[clock]);
	Serial.print(", Tol: "); Serial.print(tol);
	Serial.print(", PattLen: "); Serial.print(patternLen); Serial.print(" ");
	Serial.print(", Pulse: "); Serial.print(*first); Serial.print(", "); Serial.print(*last);
	Serial.print(", mStart: "); Serial.print(mstart);
	Serial.print(", MCD: "); Serial.print(mcDetected);


	Serial.println(); Serial.print("Signal: ");
	uint8_t idx;
	for (idx = 0; idx<messageLen; ++idx) {
		Serial.print(*(message + idx));
	}
	Serial.print(". "); Serial.print(" ["); Serial.print(messageLen); Serial.println("]");
	Serial.print("Pattern: ");
	for (uint8_t idx = 0; idx<patternLen; ++idx) {
		Serial.print(" P"); Serial.print(idx);
		Serial.print(": "); Serial.print(histo[idx]);  Serial.print("*[");
		if (pattern[idx] != 0)
		{
			Serial.print(",");
			Serial.print(pattern[idx]);
		}

		Serial.print("]");
	}
	Serial.println();
}

int8_t SignalDetectorClass::findpatt(const int val)
{
	//seq[0] = L�nge  //seq[1] = 1. Eintrag //seq[2] = 2. Eintrag ...
	// Iterate over patterns (1 dimension of array)
	for (uint8_t idx = 0; idx<patternLen; ++idx)
	{

		if (inTol(val, pattern[idx],tol))  // Skip this iteration, if seq[x] <> pattern[idx][x]
									   //if (!inTol(seq[x],pattern[idx][x-1]))  // Skip this iteration, if seq[x] <> pattern[idx][x]
		{
			return idx;
		}
	}
	// sequence was not found in pattern
	return -1;
}
/*
bool SignalDecoderClass::validSequence(const int * a, const int * b)
{
	return ((*a ^ *b) < 0); // true if a and b have opposite signs
}
*/

void SignalDetectorClass::calcHisto(const uint8_t startpos, uint8_t endpos)
{
	for (uint8_t i = 0; i<maxNumPattern; ++i)
	{
		histo[i] = 0;
	}

	if (endpos == 0) endpos = messageLen;

	for (uint8_t i = startpos; i<endpos; ++i)
	{
		histo[message[i]]++;
	}
}

bool SignalDetectorClass::getClock()
{
	// Durchsuchen aller Musterpulse und pr�ft ob darin eine clock vorhanden ist
#if DEBUGDETECT > 3
	Serial.println("  --  Searching Clock in signal -- ");
#endif
	int tstclock = -1;

	clock = -1; // Workaround for sync detection bug.

	for (uint8_t i = 0; i<patternLen; ++i) 		  // Schleife f�r Clock
	{
		//if (pattern[i][0]<=0 || pattern[i][0] > 3276)  continue;  // Annahme Werte <0 / >3276 sind keine Clockpulse
		if (tstclock == -1 && (pattern[i] >= 0) && (histo[i] > messageLen*0.17))
		{
			tstclock = i;
			continue;
		}

		if ((pattern[i] >= 0) && (pattern[i] < pattern[tstclock]) && (histo[i] > messageLen*0.17)) {
			tstclock = i;
		}
	}


	// Check Anzahl der Clockpulse von der Nachrichtenl�nge
	//if ((tstclock == 3276) || (maxcnt < (messageLen /7*2))) return false;
	if (tstclock == -1) return false;

	clock = tstclock;

	// Todo: GGf. andere Pulse gegen die ermittelte Clock verifizieren

	state = clockfound;
	return true;
}

bool SignalDetectorClass::getSync()
{
	// Durchsuchen aller Musterpulse und pr�ft ob darin ein Sync Faktor enthalten ist. Anschlie�end wird verifiziert ob dieser Syncpuls auch im Signal nacheinander �bertragen wurde
	//
#if DEBUGDETECT > 3
	Serial.println("  --  Searching Sync  -- ");
#endif

	if (state == clockfound)		// we need a clock to find this type of sync
	{
		// clock wurde bereits durch getclock bestimmt.
		for (int8_t p = patternLen - 1; p >= 0; --p)  // Schleife f�r langen Syncpuls
		{
			//if (pattern[p] > 0 || (abs(pattern[p]) > syncMaxMicros && abs(pattern[p])/pattern[clock] > syncMaxFact))  continue;  // Werte >0 oder l�nger maxfact sind keine Sync Pulse
			//if (pattern[p] == -1*maxPulse)  continue;  // Werte >0 sind keine Sync Pulse
			//if (!validSequence(&pattern[clock],&pattern[p])) continue;
			/*
			if ( (pattern[p] > 0) ||
			((abs(pattern[p]) > syncMaxMicros && abs(pattern[p])/pattern[clock] > syncMaxFact)) ||
			(pattern[p] == -maxPulse) ||
			(!validSequence(&pattern[clock],&pattern[p])) ||
			(histo[p] > 6)
			) continue;
			*/
			uint16_t syncabs = abs(pattern[p]);
			if ((pattern[p] < 0) &&
				//((abs(pattern[p]) <= syncMaxMicros && abs(pattern[p])/pattern[clock] <= syncMaxFact)) &&
				(syncabs < syncMaxMicros && syncabs / pattern[clock] <= syncMaxFact) &&
				(syncabs > syncMinFact*pattern[clock]) &&
				// (syncabs < maxPulse) &&
				//	 (validSequence(&pattern[clock],&pattern[p])) &&
				(histo[p] < 8) && (histo[p] > 1)
				//(syncMinFact*pattern[clock] <= syncabs)
				)
			{
				//if ((syncMinFact* (pattern[clock]) <= -1*pattern[p])) {//n>9 => langer Syncpulse (als 10*int16 darstellbar
				// Pr�fe ob Sync und Clock valide sein k�nnen
				//	if (histo[p] > 6) continue;    // Maximal 6 Sync Pulse  Todo: 6 Durch Formel relativ zu messageLen ersetzen

				// Pr�fen ob der gefundene Sync auch als message [clock, p] vorkommt
				uint8_t c = 0;

				//while (c < messageLen-1 && message[c+1] != p && message[c] != clock)		// Todo: Abstand zum Ende berechnen, da wir eine mindest Nachrichtenl�nge nach dem sync erwarten, brauchen wir nicht bis zum Ende suchen.

				while (c < messageLen - 1)		// Todo: Abstand zum Ende berechnen, da wir eine mindest Nachrichtenl�nge nach dem sync erwarten, brauchen wir nicht bis zum Ende suchen.
				{
					if (message[c + 1] == p && message[c] == clock) break;
					c++;
				}

				//if (c==messageLen) continue;	// nichts gefunden, also Sync weitersuchen
				if (c<messageLen - minMessageLen)
				{
					sync = p;
					state = syncfound;
					mstart = c;

#ifdef DEBUGDECODE
					//debug
					Serial.println();
					Serial.print("PD sync: ");
					Serial.print(pattern[clock]); Serial.print(", "); Serial.print(pattern[p]);
					Serial.print(", TOL: "); Serial.print(tol);
					Serial.print(", sFACT: "); Serial.println(pattern[sync] / (float)pattern[clock]);
#endif
					return true;
				}
			}
		}
	}
	sync = -1; // Workaround for sync detection bug.
	return false;
}

void SignalDetectorClass::printMsgStr(const String * first, const String * second, const String * third)
{
	Serial.print(*first);
	Serial.print(*second);
	Serial.print(*third);

}

int8_t SignalDetectorClass::printMsgRaw(uint8_t m_start, const uint8_t m_end, const String * preamble, const String * postamble)
{
	Serial.print(*preamble);
	//String msg;
	//msg.reserve(m_end-mstart);
	byte crcv = 0x00;
	for (; m_start <= m_end; m_start++)
	{
		//msg + =message[m_start];
		//Serial.print((100*message[m_start])+(10*message[m_start])+message[m_start]);
		Serial.print(message[m_start]);
#ifndef ARDUSIM
		//crcv = _crc_ibutton_update(crcv, message[m_start]);
#endif
	}
	//Serial.print(msg);
	Serial.print(*postamble);
	return crcv;
	//printMsgStr(preamble,&msg,postamble);}
}






/*
********************************************************
************* Manchester DECODER class ***************
********************************************************
*/
/** @brief (Constructor for Manchester decoder. ref= object of type patternDetecor which is calling the manchester decoder)
*
* Initialisation of class MancheserpatternDecoder
*/
/*
ManchesterpatternDecoder::ManchesterpatternDecoder(signalDecoder *ref_dec)
{
pdec = ref_dec;
//ManchesterBits->new BitStore(1); // use 1 Bit for every value stored, reserve 30 Bytes = 240 Bits
reset();
}*/
/** @brief (one liner)
*
* (documentation goes here)
*/
ManchesterpatternDecoder::~ManchesterpatternDecoder()
{
	//delete ManchesterBits->

}



/** @brief (Resets internal vars to defaults)
*
* Reset internal vars to defaults. Called after error or when finished
*/
void ManchesterpatternDecoder::reset()
{
	
	longlow =   -1;
	longhigh =  -1;
	shortlow =  -1;
	shorthigh = -1;
	
	bool mc_start_found = false;
	bool mc_sync = false;
	minbitlen = 20; // Set defaults
	ManchesterBits.reset();
}
/** @brief (Sets internal minbitlen to new value)
*
* (documentation goes here)
*/
void ManchesterpatternDecoder::setMinBitLen(const uint8_t len)
{
	minbitlen = len;
}


/** @brief (Returns true if given pattern index matches a long puls index)
*
* (documentation goes here)
*/
const bool ManchesterpatternDecoder::isLong(const uint8_t pulse_idx)
{
	return (pulse_idx == longlow || pulse_idx == longhigh);
}

/** @brief (Returns true if given pattern index matches a short puls index)
*
* (documentation goes here)
*/

const bool ManchesterpatternDecoder::isShort(const uint8_t pulse_idx)
{
	return (pulse_idx == shortlow || pulse_idx == shorthigh);
}

/** @brief (Converts decoded manchester bits in a provided string as hex)
*
* ()
*/
void ManchesterpatternDecoder::getMessageHexStr(String *message)
{
	char hexStr[] = "00" ; // Not really needed

	message->reserve((ManchesterBits.valcount /4)+2);
	if (!message)
		return;
	uint8_t idx;
	// Bytes are stored from left to right in our buffer. We reverse them for better readability
	for ( idx = 0; idx <= ManchesterBits.bytecount-1; ++idx) {
		//Serial.print(getMCByte(idx),HEX);
		//sprintf(hexStr, "%02X",reverseByte(ManchesterBits->>getByte(idx)));
		//Serial.print(".");
		sprintf(hexStr, "%02X", getMCByte(idx));
		message->concat(hexStr);
		//Serial.print(hexStr);
	}
	
	sprintf(hexStr, "%01X", getMCByte(idx) >> 4 & 0xf);
	message->concat(hexStr);
	if (ManchesterBits.valcount % 8 > 4)
	{
		sprintf(hexStr, "%01X", getMCByte(idx) & 0xF);
		message->concat(hexStr);
	}

	//Serial.println();

}

/** @brief (one liner)
*
* (documentation goes here)
*/
void ManchesterpatternDecoder::getMessagePulseStr(String* str)
{
	str->reserve(32);
	if (!str)
		return;

	str->concat("LL="); str->concat(pdec->pattern[longlow]); str->concat(SERIAL_DELIMITER);
	str->concat("LH="); str->concat(pdec->pattern[longhigh]); str->concat(SERIAL_DELIMITER);
	str->concat("SL="); str->concat(pdec->pattern[shortlow]); str->concat(SERIAL_DELIMITER);
	str->concat("SH="); str->concat(pdec->pattern[shorthigh]); str->concat(SERIAL_DELIMITER);
}

/** @brief (one liner)
*
* (documentation goes here)
*/
void ManchesterpatternDecoder::getMessageClockStr(String* str)
{
	str->reserve(7);
	if (!str)
		return;

	str->concat("C="); str->concat(clock); str->concat(SERIAL_DELIMITER);
}

void ManchesterpatternDecoder::getMessageLenStr(String* str)
{

	str->concat("L="); str->concat(ManchesterBits.valcount); str->concat(SERIAL_DELIMITER);
}


/** @brief (retieves one Byte out of the Bitstore for manchester decoded bits)
*
* (Returns a comlete byte from the pattern store)
*/

unsigned char ManchesterpatternDecoder::getMCByte(const uint8_t idx) {

	return ManchesterBits.getByte(idx);
}




/** @brief (Decodes the manchester pattern to bits. Returns true on success and false on error )
*
* (Call only after ismanchester returned true)
*/

const bool ManchesterpatternDecoder::doDecode() {
	//Serial.print("bitcnt:");Serial.println(bitcnt);

	uint8_t i = 0;
	pdec->m_truncated = false;
//	bool mc_start_found = false;
//	bool mc_sync = false;
	pdec->mstart = 0;
#ifdef DEBUGDECODE
	Serial.print("mlen:");
	Serial.print(pdec->messageLen);
	Serial.print(":mstart: ");
	Serial.print(pdec->mstart);
#endif
	char  lastbit;

	while (i < pdec->messageLen)
	{
		
		// Start vom MC Signal suchen
		if (mc_start_found == false && (isLong(pdec->message[i]) || isShort(pdec->message[i])))
		{
			pdec->mstart = i;
			mc_start_found = true;
		}
		// Sync to a long pulse to detect 0 / 1 proper
		if (mc_start_found && !mc_sync)
		{
			while (!isLong(pdec->message[i]) && i < pdec->messageLen) {
				i++;
			}
			if (i < pdec->messageLen) {
				lastbit = (char)((unsigned int)pdec->pattern[pdec->message[i]] >> 15);
				uint8_t z = i - pdec->mstart;
				if ((z < 1) or ((z % 2) == 0))
					i = pdec->mstart;
				else
					i = pdec->mstart + 1;
				//ManchesterBits->addValue((lastbit));
				mc_sync = true;
				//i++;
			}
		}

		// Decoding occures here
		if (mc_sync && mc_start_found)
		{
			if (isLong(pdec->message[i])) {
				//ManchesterBits->addValue(!(pdec->pattern[pdec->message[i]][0] >>15)); // Check if bit 16 is set
				//ManchesterBits->addValue(1 ^ ((unsigned int)pdec->pattern[pdec->message[i]][0] >> 15)));
				lastbit = lastbit ^ 1;
#ifdef DEBUGDECODE
				//ManchesterBits->addValue(lastbit);
				Serial.print("L");
#endif
			}
			else if (isShort(pdec->message[i]) && i < pdec->messageLen - 1 && isShort(pdec->message[i + 1]))
			{

				i++;
				//ManchesterBits->addValue(!(pdec->pattern[pdec->message[i+1]][0] >>15)); // Check if bit 16 is set
				// ManchesterBits->addValue(1 ^ ((unsigned int)pdec->pattern[pdec->message[i]][0] >> 15)));
#ifdef DEBUGDECODE
				Serial.print("SS");
#endif

			}
			else { // Found something that fits not to our manchester signal
#ifdef DEBUGDECODE
				Serial.print("H(");
				Serial.print("vcnt:");
				Serial.print(ManchesterBits.valcount);
#endif

				   //if (i < pdec->messageLen-minbitlen)
				if (ManchesterBits.valcount < minbitlen)
				{
					if (isShort(pdec->message[i]) && i < pdec->messageLen - 1 && !isShort(pdec->message[i + 1])) {
						// unequal number of short pulses. Restart, but one pulse ahead i is incremented at end of while loop
						i = pdec->mstart;
					}
					//pdec->mstart=i;
					mc_start_found = false; // Reset to find new starting position
					mc_sync = false;
#ifdef DEBUGDECODE
					Serial.print(":RES:");
#endif
					ManchesterBits.reset();

				} else {
					pdec->mend = i;
					if (isShort(pdec->message[i]) && i == maxMsgSize - 1) {
						pdec->mend--;
					}
#ifdef DEBUGDECODE
					Serial.print(":mpos:");
					Serial.print(i);
					Serial.print(":mstart:");
					Serial.print(pdec->mstart);
					Serial.print(":mend:");
					Serial.print(pdec->mend);
					Serial.print(":found:");
#endif
					pdec->bufferMove(pdec->mend);
					pdec->m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
												//if (i+minbitlen > pdec->messageLen)
					/*
					if ( isShort(pdec->message[pdec->messageLen]) )
					{

						pdec->mcDetected = true;
						return false;
					}
					*/
					return (ManchesterBits.valcount >= minbitlen);  // Min 20 Bits needed
				}
#ifdef DEBUGDECODE
				Serial.print(")");
#endif
			}
			if (mc_sync)
				ManchesterBits.addValue(lastbit);


		}
		//Serial.print(" S MC ");
		i++;
	}
	pdec->mend = i;

#ifdef DEBUGDECODE
	Serial.print(":mpos:");
	Serial.print(i);
	Serial.print(":mstart:");
	Serial.print(pdec->mstart);
	Serial.print(":mend:");
	Serial.print(pdec->mend);
	Serial.print(":vcnt:");
	Serial.print(ManchesterBits.valcount);
	Serial.print(":bfin:");
#endif


	// Check if last entry in our message array belongs to our manchester signal
	if (i == maxMsgSize && i == pdec->messageLen  && pdec->mstart > 1 && ManchesterBits.valcount > minbitlen / 2)
	{
#ifdef DEBUGDECODE
		Serial.print(":bmove:");
#endif

		pdec->bufferMove(pdec->mstart);
		pdec->m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
		
		pdec->mcDetected = true;
		return false;
	}
	// Buffer is full with mc signal, so we clear the buffer and caputre additional signaldata
	else if (i == maxMsgSize && i == pdec->messageLen && pdec->mstart == 0)
	{
		pdec->mcDetected = true;
		pdec->messageLen = 0;
		pdec->m_truncated = true;  // Flag that we truncated the message array and want to receiver some more data
#ifdef DEBUGDECODE
		Serial.print(":bflush:");
#endif
		return false;
	}

	return (ManchesterBits.valcount >= minbitlen);  // Min 20 Bits needed, then return true, otherwise false

													//Serial.print(" ES MC ");
}

/** @brief (Verifies if found signal data is a valid manchester signal, returns true or false)
*
* (Check signal based on patternLen, histogram and pattern store for valid manchester style.Provides key indexes for the 4 signal states for later decoding)
*/

const bool ManchesterpatternDecoder::isManchester()
{
	// Durchsuchen aller Musterpulse und pr�ft ob darin eine clock vorhanden ist
#if DEBUGDETECT >= 1
	Serial.print("  --  chk MC -- ");
#endif
	if (pdec->patternLen < 4)	return false;

	int tstclock = -1;

	uint8_t pos_cnt = 0;
	uint8_t neg_cnt = 0;
	uint8_t equal_cnt = 0;
	const uint8_t minHistocnt = pdec->messageLen*0.04;

	for (uint8_t i = 0; i< pdec->patternLen; i++)
	{
#if DEBUGDETECT >= 1
		Serial.print(i); Serial.print(" ");
#endif

		if (pdec->histo[i] < minHistocnt) continue;		// Skip this pattern, due to less occurence in our message
		const int aktpulse = pdec->pattern[i];
		//if (longlow == -1)
		//    longlow=longhigh=shortlow=shorthigh=i;  // Init to first valid mc index to allow further ajustment


		if (aktpulse > 0)
		{
			equal_cnt += pdec->histo[i];
			pos_cnt++;
			tstclock += aktpulse;

			longhigh = longhigh == -1 || pdec->pattern[longhigh] < aktpulse ? i : longhigh;
			shorthigh = shorthigh == -1 || pdec->pattern[shorthigh] > aktpulse ? i : shorthigh;

		}
		else {
			equal_cnt -= pdec->histo[i];
			neg_cnt++;

			longlow = longlow == -1 || pdec->pattern[longlow] > aktpulse ? i : longlow;
			shortlow = shortlow == -1 || pdec->pattern[shortlow] < aktpulse ? i : shortlow;
		}

	}
#if DEBUGDETECT >= 1
	Serial.print("equalcnt: "); Serial.print(equal_cnt);
#endif

	if (equal_cnt > pdec->messageLen*0.02) return false;
#if DEBUGDETECT >= 1
	Serial.print("  MC equalcnt matched");
#endif

	if (neg_cnt != pos_cnt) return false;  // Both must be 2
#if DEBUGDETECT >= 1
	Serial.print("  MC neg and pos pattern cnt is equal");
#endif

	tstclock = tstclock / 3;
#if DEBUGDETECT >= 1
	Serial.print("  tstclock: "); Serial.print(tstclock);
#endif
	clock = tstclock;
	//	dclock=clock*2;

#if DEBUGDETECT >= 1
	Serial.print(" MC LL:"); Serial.print(longlow);
	Serial.print(", MC LH:"); Serial.print(longhigh);

	Serial.print(", MC SL:"); Serial.print(shortlow);
	Serial.print(", MC SH:"); Serial.print(shorthigh);
	Serial.println("");
#endif
	if ((longlow == -1) || (shortlow == -1) || (longlow == shortlow) || (longhigh == -1) || (shorthigh == -1) || (longhigh == shorthigh)) return false; //Check if the indexes are valid
	
	if ((longlow == longhigh) || (shortlow == shorthigh) || (longlow == shortlow) || (longhigh == shorthigh) || (longlow == shorthigh) || (longhigh == shortlow)) return false; //Check if the indexes are valid

#if DEBUGDETECT >= 1
	Serial.println("  -- MC found -- ");
#endif

	return true;
}

