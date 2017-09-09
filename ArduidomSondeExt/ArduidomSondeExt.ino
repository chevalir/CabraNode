//


		// @TODO potencial infini LOOP  looking for @TODO


//------------------------------------------------------------------------------
//  ____________ _   _           _
//  | ___ \  ___| \ | |         | |
//  | |_/ / |_  |  \| | ___   __| | ___
//  |    /|  _| | . ` |/ _ \ / _` |/ _ \
//  | |\ \| |   | |\  | (_) | (_| |  __/
//  \_| \_\_|   \_| \_/\___/ \__,_|\___|
//
//
// Author  : chevalir
// version : ExtV5
/*
http://patorjk.com/software/taag/#p=display&v=3&c=c%2B%2B&f=Doom&t=TypeHere
*/
//------------------------------------------------------------------------------
#include <Arduino.h>

////////////////////////////////////////////////////////////////////////////////
#include "Arduidom_Radio.h"
#include <avr/wdt.h> // watchDog API


// DEBUG LEVEL
#define LOG 1
#define LOG_DEBUG 1
#define LOG_INFO 1
#define LOG_WARNING 1


#define NDSPROBE 2 // Number of DS18B20 connected to
#define NDHT 1 // Number of DHT22 connected to
const unsigned int READ_ERROR = 9999;

// GLOBAL constantes
#define PROBE_ID 0 // First ID of probe 
#define STATE_ID 0 // First ID of status 

#define RFTX_PIN 7 // D7 Some time on D8 depend of the probe 433 transmiter
#define ONEWIRE_PIN 9 // DS18B20 temperature probe.
#define DHT_PIN 2           // DTH 22 Probe
//#define ledPin 13
#define STATUS_PIN 3  // Physical pin on arduino
#define STATUS_VPIN 0 // Virtual Pin on Arduidom

// RADIO RF 433
const byte RADIO_REPEATS = 10; // Nombre de repetitions des messages Radio
RCSwitch mySwitch = RCSwitch();

volatile int aState = 0;         // variable for reading the status PIN
volatile int lastState = 0;

// DHT initialisation
#if (NDHT > 0)
#include "DHT.h"
DHT dhtprobe(DHT_PIN, DHT22);
#endif

// Dallas DS18B20 temperature sensor bus init
#if (NDSPROBE > 0)
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature sensors(&oneWire);
// int LastDSValue[NDSPROBE];
byte nbTemperatureSensor = 0;
#endif

// minimum gap to send value 10 for 0.1°C, and 100 for 1%
int gapMin[NDSPROBE + NDHT*2]        = { 10, 10, 10, 100};
// offset to add to PROBE_ID
int probeIDOffset[NDSPROBE + NDHT*2] = {0, 1 , 2, 3};
// type off probe D for DS18B20, T for DHT22 Temp, H = DHT22 Humidity
int probeType[NDSPROBE + NDHT*2] = {'D','D','T','H'};
int lastValue[NDSPROBE + NDHT*2]   = {9000, 9000, 9000, 9000}; // last temperature and humidity vales sent
byte skeepCount[NDSPROBE + NDHT*2] = {11, 11, 11, 11};  // count send skeeped due to no change set to 11 to
                                                        // force send after setup

unsigned long tempLastCheckProbe = 0 ;
unsigned long tempLastSendState = 0 ;

unsigned long checkProbFreq = 5*60000; //N * 1 minutes
const byte SKEEP_MAX = 10;  // max time to send update is checkProbFreq * SKEEP_MAX

//bool itsTimeToSend = true;
bool itsTimeForTemp = false;

unsigned long stateSendFreq = 2000; // frequency to send temperatures, in milli
volatile bool isStatusChanged = true;
long currentMillis, oldMillis = 0;

//            _
//           | |
//   ___  ___| |_ _   _ _ __
//  / __|/ _ \ __| | | | '_ \
//  \__ \  __/ |_| |_| | |_) |
//  |___/\___|\__|\__,_| .__/
//                     | |
//                     |_|
void setup() {
	wdDisable();	
	#if defined(LOG)	
	Serial.begin(115200); // Init du Port serie/USB
	Serial.setTimeout(5); // Timeout 5ms
	#endif

	pinMode(ONEWIRE_PIN, INPUT);
	#if defined(ledPin)
	pinMode(ledPin, OUTPUT);
	#endif
	pinMode(STATUS_PIN, INPUT_PULLUP);

	pinMode(RFTX_PIN, OUTPUT);
	mySwitch.enableTransmit(RFTX_PIN); // Transmission sur Pin
	mySwitch.setRepeatTransmit(RADIO_REPEATS); // Repete x fois le message

	//--------------------------------------------------------------------------------------------------------------------------------------------------
	#if (NDSPROBE > 0)
	sensors.begin();
	nbTemperatureSensor = sensors.getDeviceCount();
	#if defined(LOG_INFO)		
	Serial.print("Nb DS Sensor:"); Serial.println(nbTemperatureSensor);
	#endif
	// just to avoid probleme with some sensor where the first value returned is not correct
	for (int ploop = 0 ; ploop < NDSPROBE; ploop++) {
		getDSTemperature(ploop);
	}
	#endif
	#if (NDHT > 0)	
	dhtprobe.begin();
	#endif	

	attachInterrupt(digitalPinToInterrupt(STATUS_PIN), aStatusPinISR, CHANGE);

	// Start
	#if defined(LOG)		
	logBuildVersion();
	#endif
	Serial.print( checkProbFreq, DEC  );
	Serial.println("  -------------checkProbFreq");

	#if defined(ledPin)
	blinkLed(2, 10);
	#endif
	
	wdEnable();
	// FD:12802190:A:100:P:4
	
/*	for (int i = 1; i <= RADIO_REPEATS; i++) {
		mySwitch.send(12902190, 0, 0, 0);
	}
	
	delay(5000);
	for (int i = 1; i <= RADIO_REPEATS; i++) {
		mySwitch.send(12902190, 0, 0, 1);
	}*/
	
} // End Of void setup()




//                   _         _
//                  (_)       | |
//   _ __ ___   __ _ _ _ __   | |     ___   ___  _ __
//  | '_ ` _ \ / _` | | '_ \  | |    / _ \ / _ \| '_ \
//  | | | | | | (_| | | | | | | |___| (_) | (_) | |_) |
//  |_| |_| |_|\__,_|_|_| |_| \_____/\___/ \___/| .__/
//                                              | |
//

void loop(){
	wdReset();	
	currentMillis = millis();
	#if defined(LOG_INFO)
		long diff = currentMillis - oldMillis;
		if (diff > 10) {
			Serial.print( currentMillis - oldMillis );
			Serial.println("  -------------diff time");
		}
		oldMillis = currentMillis;
	#endif
	
	if (lastState != aState) {
		delay(500);
		manageInput();
	}
	wdReset();	

	
	// manage status to send
	if ((currentMillis - tempLastSendState) > stateSendFreq && isStatusChanged) {
		send2RF(true , STATUS_VPIN, aState, PROBE_ID, false);
		isStatusChanged = false;
		tempLastSendState = currentMillis;
	}
	wdReset();	
	
	
	if ((currentMillis - tempLastCheckProbe) > checkProbFreq ) {		
	#if defined(LOG_INFO)		
		Serial.println("*********************** tempLastCheckProbe *****************************"); 
		Serial.println();
	#endif	
		sendAllProbeValues(NDSPROBE + NDHT*2);
		tempLastCheckProbe = currentMillis;
	}
}


//   _     _       _    _              _
//  | |   (_)     | |  | |            | |
//  | |__  _ _ __ | | _| |     ___  __| |
//  | '_ \| | '_ \| |/ / |    / _ \/ _` |
//  | |_) | | | | |   <| |___|  __/ (_| |
//  |_.__/|_|_| |_|_|\_\_____/\___|\__,_|
//
//
#if defined(ledPin)
void blinkLed(int repeat, int time) {
	for (int i = 0; i < repeat; i++) {
		delay(time);
		digitalWrite(ledPin, HIGH);
		delay(time);
		digitalWrite(ledPin, LOW);
	}
}
#endif


//   _____                  _
//  |_   _|                | |
//    | | _ __  _ __  _   _| |_
//    | || '_ \| '_ \| | | | __|
//   _| || | | | |_) | |_| | |_
//   \___/_| |_| .__/ \__,_|\__|
//             | |
//             |_|

void manageInput() {
	// delay(5000); // wait 5s in case of the signal comes from the Arduidom
	// in this case arduino need time to be able to receive message.
	byte bState = digitalRead(STATUS_PIN);
	if (lastState != bState) {
		#if defined (LOG_INFO)
		Serial.print(" BUTTON="); Serial.println( bState );
		#endif
		lastState = bState;
		isStatusChanged=true;
	}
}

void aStatusPinISR() {
	aState = digitalRead(STATUS_PIN);
	delay(10);
}



//   _____                                   _
//  |_   _|                                 | |
//    | | ___ _ __ ___  _ __   ___ _ __ __ _| |_ _   _ _ __ ___
//    | |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ _` | __| | | | '__/ _ \
//    | |  __/ | | | | | |_) |  __/ | | (_| | |_| |_| | | |  __/
//    \_/\___|_| |_| |_| .__/ \___|_|  \__,_|\__|\__,_|_|  \___|
//                     | |
//                     |_|
/**
   TorH : true for temperature, false for humidity
**/
void sendAllProbeValues(byte nbTotalProbe) {
	for ( int numProbe = 0;  numProbe <nbTotalProbe ; numProbe++) {
		sendProbeValues(numProbe);
	}
}

void sendProbeValues(int lProbeNum) {
	// sent only when change detected 

	#if defined (LOG_DEBUG)
	Serial.print("sendProbeValues #"); Serial.print(lProbeNum);
	Serial.print(" Type="); Serial.println(probeType[lProbeNum]);
	#endif

	int newValue = READ_ERROR;

	switch (probeType[lProbeNum]) {
	case 'H' :
		newValue = getDHT22Humi();
		break;
	case 'T' :
		newValue = getDHT22Temp();
		break;
	case 'D' :
		newValue = getDSTemperature(lProbeNum);
		break;
	}


	#if defined (LOG_DEBUG)
	Serial.print("newValue="); Serial.print(newValue);
	Serial.print(" lastValue="); Serial.println(lastValue[lProbeNum]);
	#endif	


	long gap = 0;
	if ((lastValue[lProbeNum] != newValue)) {
		gap = abs(lastValue[lProbeNum] - newValue);
		#if defined (LOG_DEBUG)
		Serial.print("GapMin="); Serial.print(gapMin[lProbeNum]);
		Serial.print(" gap="); Serial.println(gap);
		#endif	
	}
	if ( gap >= 3*gapMin[lProbeNum] // send temp when change >= 0.3°
	     || (gap >= gapMin[lProbeNum] && skeepCount[lProbeNum] > (SKEEP_MAX/2) ) // send low change every 10 min if any (depend of conf)
	     || skeepCount[lProbeNum] > SKEEP_MAX // send value every 20 min ( depend of conf )
	   ) {
		lastValue[lProbeNum] = sendProbeValue(lProbeNum, newValue, true);
		Serial.print(" ret="); Serial.println(lastValue[lProbeNum]);
		skeepCount[lProbeNum] = 0;
		
	} else {
		#if defined (LOG_DEBUG)
		Serial.print("No Change / skeepCount="); Serial.println(skeepCount[lProbeNum]);
		#endif	
		skeepCount[lProbeNum] += 1;
	}

}



/*******************************************************************************
*******************************************************************************/
#if (NDSPROBE > 0)
float getDSTemperature(byte probeNum) {
	// *** MORE THAN ONE RETURN ****
	float rawValue = NAN;
	if ( nbTemperatureSensor > probeNum ) {
		sensors.requestTemperatures();
		rawValue = sensors.getTempCByIndex(probeNum);
	}

	if ( isnan(rawValue) || rawValue < -10 || rawValue == 85 || rawValue > 125 ) {
		#if defined(LOG_WARNING)		
		Serial.println("DS #") ; Serial.print(probeNum) ;
		Serial.println(" off or error");
		#endif
		return READ_ERROR;
	}
	#if defined(LOG_DEBUG)		
	Serial.print("DS #") ; Serial.print(probeNum); 
	Serial.print(" t="); Serial.println(rawValue);
	#endif
	return rawValue*100;
}
#endif

#if (NDHT > 0)
/*******************************************************************************
*******************************************************************************/
//              _  ______ _   _ _____ _____  _____
//             | | |  _  \ | | |_   _/ __  \/ __  \
//    __ _  ___| |_| | | | |_| | | | `' / /'`' / /'
//   / _` |/ _ \ __| | | |  _  | | |   / /    / /
//  | (_| |  __/ |_| |/ /| | | | | | ./ /___./ /___
//   \__, |\___|\__|___/ \_| |_/ \_/ \_____/\_____/
//    __/ |
//   |___/

/****
   TorH : true for Temperature, false for Humidity
**/
int getDHT22Humi() {
	return getDHT22Value(false);
}


int getDHT22Temp() {
	return getDHT22Value(true);
}


int getDHT22Value(boolean TorH) {
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	float rawValue = TorH ? dhtprobe.readTemperature() : dhtprobe.readHumidity();
	byte retry = 0;
	const byte RETRY_MAX = 10;

	while ( isnan(rawValue) && retry < RETRY_MAX) {		
		delay(100);
		#if defined(LOG_WARNING)		
		Serial.print("DHT read retry temp="); Serial.println(TorH);
		#endif
		rawValue = TorH ? dhtprobe.readTemperature() : dhtprobe.readHumidity();
		retry++;
	}
	#if defined(LOG_INFO)		
	Serial.print("DHT v:"); Serial.println(rawValue);
	#endif

	if (isnan(rawValue)) {
		return READ_ERROR;
	}
	return rawValue*100;
}
#endif


//                      _______          _           _   _       _
//                     | | ___ \        | |         | | | |     | |
//   ___  ___ _ __   __| | |_/ / __ ___ | |__   ___ | | | | __ _| |_   _  ___
//  / __|/ _ \ '_ \ / _` |  __/ '__/ _ \| '_ \ / _ \| | | |/ _` | | | | |/ _ \
//  \__ \  __/ | | | (_| | |  | | | (_) | |_) |  __/\ \_/ / (_| | | |_| |  __/
//  |___/\___|_| |_|\__,_\_|  |_|  \___/|_.__/ \___| \___/ \__,_|_|\__,_|\___|
//
//
int sendProbeValue( byte aProbeNum, int aValue, bool aSend ) {
	int decValueD;
	bool positive = aValue > 0;
	int tempValue;
	#if defined(LOG_INFO)	
	Serial.print(">>Probe:" ); Serial.print(aProbeNum);
	Serial.print(" v:"); Serial.println(aValue, DEC);
	#endif

	// bad values returned by the sensor >-10 or 85
	if ( aValue == READ_ERROR ) {
		#if defined(LOG_INFO)		
		#endif
		if ( lastValue[aProbeNum] == 9000 ) {
			Serial.println("READ_ERROR 99");
			tempValue = 99;
		} else {
			Serial.print("READ_ERROR 98"); Serial.println(lastValue[aProbeNum]);
			tempValue = 99;
		}
			
		//tempValue = 99;
		decValueD = 9;
		positive = false;
	} else {
		// arround calculation  ex aValue = 1935 -->  19.4
		tempValue = aValue / 100; // remove decimal part tempValue = 19
		decValueD = (aValue - tempValue*100)/10; // decValueD (1935 - 1900)/10 = 3
		// decValueD= 3   tempValue = 19 -> now
		int decValueU = aValue - (tempValue*100) - (decValueD*10) ; // 1935 - 19*100 - 3*10
		if (decValueU > 4) {
			++decValueD;
		}
	}
	// return temperature to -0 in case of error
	#if defined(LOG_INFO)		
	//Serial.print("Probe sensor Value "); Serial.println( aValue, DEC);
	Serial.print(" ID: " ); Serial.print( PROBE_ID+aProbeNum, DEC);
	Serial.print(" value: " );
	Serial.print(tempValue); Serial.print("."); Serial.print(decValueD);
	Serial.print(" Signe: " ); Serial.println( positive, DEC);
	#endif
	if ( aSend ) {
		send2RF(false , tempValue, decValueD, PROBE_ID+aProbeNum, positive);
	}
	return tempValue*100;
}




//                      _  _____ ____________
//                     | |/ __  \| ___ \  ___|
//   ___  ___ _ __   __| |`' / /'| |_/ / |_
//  / __|/ _ \ '_ \ / _` |  / /  |    /|  _|
//  \__ \  __/ | | | (_| |./ /___| |\ \| |
//  |___/\___|_| |_|\__,_|\_____/\_| \_\_|
//
//
/*******************************************************************************
*******************************************************************************/
void send2RF(boolean aFlagGroup, int aValue, int aDecValue, byte aReceiver, boolean aFlagOnOff ) {
	unsigned long senderID = 9999000 + (aValue*10)+aDecValue;
	#if defined(LOG_DEBUG)
	Serial.print("  >>");
	logRFMessage(senderID, aFlagGroup,aReceiver, aFlagOnOff);
    #endif 

	// Serial.println("FAKE mySwitch.send");
	for (int i = 1; i <= RADIO_REPEATS; i++) {
		mySwitch.send(senderID, aFlagGroup, aReceiver, aFlagOnOff);
	}
}

//                 _       _    ______            
//                | |     | |   |  _  \           
//  __      ____ _| |_ ___| |__ | | | |___   __ _ 
//  \ \ /\ / / _` | __/ __| '_ \| | | / _ \ / _` |
//   \ V  V / (_| | || (__| | | | |/ / (_) | (_| |
//    \_/\_/ \__,_|\__\___|_| |_|___/ \___/ \__, |
//                                           __/ |
//                                          |___/ 
void wdDisable() {
	// immediately disable watchdog timer so set will not get interrupted
	wdt_reset();
	wdt_disable();

}

void wdEnable() {
	// enable the watchdog timer. There are a finite number of timeouts allowed (see wdt.h).
	// Notes I have seen say it is unwise to go below 250ms as you may get the WDT stuck in a
	// loop rebooting.
	// The timeouts I'm most likely to use are:
	// WDTO_1S
	// WDTO_2S
	// WDTO_4S
	// WDTO_8S
	
	delay(2000);
	wdt_enable(WDTO_8S);

}

void wdReset() {
	wdt_reset();
}



//  ______       _ _     _     ___   _               _
//  | ___ \     (_) |   | |   / / | | |             (_)
//  | |_/ /_   _ _| | __| |  / /| | | | ___ _ __ ___ _  ___  _ __
//  | ___ \ | | | | |/ _` | / / | | | |/ _ \ '__/ __| |/ _ \| '_ \
//  | |_/ / |_| | | | (_| |/ /  \ \_/ /  __/ |  \__ \ | (_) | | | |
//  \____/ \__,_|_|_|\__,_/_/    \___/ \___|_|  |___/_|\___/|_| |_|
//
//
#if defined (LOG)
void logBuildVersion (void) {
	String the_path = __FILE__;
	int slash_loc = the_path.lastIndexOf('/');
	String the_cpp_name = the_path.substring(slash_loc+1);
	int dot_loc = the_cpp_name.lastIndexOf('.');
	String the_sketchname = the_cpp_name.substring(0, dot_loc);
	Serial.print("\nSketch: "); Serial.print(the_sketchname);
	Serial.print(" build:"); Serial.print(__DATE__);
	Serial.print(" / "); Serial.print(__TIME__);
	Serial.println("\n end setup");
}

/*******************************************************************************
*******************************************************************************/
void logRFMessage(unsigned long aSender, byte aGroup, byte aReceptor, byte aOnOff ) {
	Serial.print(aSender);
	Serial.print(" G:");Serial.print(aGroup);
	Serial.print(" R:");Serial.print(aReceptor);
	Serial.print(" o:");Serial.println(aOnOff);
}
#endif
// do: CPzzrtyyooiizzzzzzzzzzcczzzzdczzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz


