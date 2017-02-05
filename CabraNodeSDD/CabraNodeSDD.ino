//------------------------------------------------------------------------------
//   _____       _               _   _           _      _________________
//  /  __ \     | |             | \ | |         | |    /  ___|  _  \  _  \
//  | /  \/ __ _| |__  _ __ __ _|  \| | ___   __| | ___\ `--.| | | | | | |
//  | |    / _` | '_ \| '__/ _` | . ` |/ _ \ / _` |/ _ \`--. \ | | | | | |
//  | \__/\ (_| | |_) | | | (_| | |\  | (_) | (_| |  __/\__/ / |/ /| |/ /
//   \____/\__,_|_.__/|_|  \__,_\_| \_/\___/ \__,_|\___\____/|___/ |___/
//
// Cabra Node salle de bain la Chèvre ....
//
// Author  : chevalir
// version : SDB
//------------------------------------------------------------------------------
/*
 http://patorjk.com/software/taag/#p=display&v=3&c=c%2B%2B&f=Doom&t=TypeHere
*/
//------------------------------------------------------------------------------
#include <Arduino.h>
#include <avr/wdt.h> // watchDog API

////////////////////////////////////////////////////////////////////////////////
#include "Arduidom_Radio.h"
#include <EEPROMex.h>

// DEBUG LEVEL
#define LOG 1
#define LOG_DEBUG 1
#define LOG_INFO 1
// define LOG_WARNING 1


#define NDSPROBE 0 // Number of DS18B20 connected to
#define NDHT 1 // Number of DHT22 connected to
const unsigned int READ_ERROR = 9999;


// GLOBAL constantes
const byte PROBE_ID = 6; // ID of probe beetween 0 and 15
const byte DEVICE_STATE_ID = 5; // first state identifier added to device number

// PIN constantes
#define RFRX_PIN 2 		  // RF433 receiver
// #define BT_VMC_PIN 12      
#define BT_LIGHT_PIN 10
// $$$$$$$$$$$$$*************** 12 A REMETRE A 13  /!\    #define BT_MIRROR_PIN 13
#define BT_MIRROR_PIN 13
// #define BT_WC_PIN 11

// $$$$$$$$$$$$$*************** 7 A REMETRE  8 /!\ #define RFTX_PIN 8 		  
// D7 or D8 depend of the probe RF433 transmiter
#define RFTX_PIN 8


#define DHT_PIN 9           // DTH 22 Probe
// $$$$$$$$$$$$$***************13 A REMETRE à A4  /!\ llll
#define LED_PIN A4
#define RS_VMC_PIN 4  	  // RELAIS IN1
#define RS_LIGHT_PIN 5   // RELAUF IN2
#define RS_MIRROR_PIN 6  // RELAIS IN3
// #define RS_WC_LIGHT_PIN 7 // RELAIS IN4

// RADIO RF 433
const byte RADIO_REPEATS = 5; // Nombre de repetitions des messages Radio
const long HE_IDE = 19940334; // default HomeEasy ID ( ID oF DIO REF:54791 ;) )
// const long HE_IDE = 4988678; // default HomeEasy ID


//  RCSwitch lib initialisation
RCSwitch mySwitch = RCSwitch();
// HomeEasy protocole variable
unsigned long RFLastSender = 0; // Valeur du Data Recu par 433
unsigned long RFLastAddr = 999; // Contains the target Id and the action to do 
// ON/OFF initialized to "impossible" value 
byte RFOnOff = 0;
byte RFLastReceptor = 0;


const byte NB_BUTTON = 4; // Total number of button
const byte NB_DEVICE = 3; // Total number of device
const byte DELAY_REBOUND = 50;
const byte VMC = 0; // VMC table index
const byte LIGHT = 1; // Ligth table index
const byte MIRROR = 2; // mirror light table index
const byte WC = 3; // WC table index
const bool buttonExist[NB_BUTTON] = {false, true, true, false};

// DHT initialisation
#if (NDHT > 0)
#include "DHT.h"
DHT dhtprobe(DHT_PIN, DHT22);
// byte tempDHT = 0;
// byte humiDHT = 1;
int lastValue[2] = {-99, -99}; // last temperature and humidity vales sent
byte skeepCount[2] = {0,0};  // count send skeeped due to no change
int gapMin[2] = {10, 100}; // 10 for 0.1°C, and 100 for 1%
#endif

// Dallas DS18B20 temperature sensor bus init
#if (NDSPROBE > 0)
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature sensors(&oneWire);
#endif


unsigned long tempLastCheckProbe = 0 ;
unsigned long tempLastSendState = 0 ;

unsigned long stateSendFreq = 3000; // frequency to send status, in milli
unsigned long checkProbFreq = 5*60000; //5 * 1 minutes
const byte SKEEP_MAX = 3;

// Command group
// 0 = inputPin, 1
byte buttonPins[NB_BUTTON];
byte buttonLastState[NB_BUTTON];
byte devicePin[NB_DEVICE];
byte deviceState[NB_DEVICE];
volatile bool isStatusChanged[NB_DEVICE];
byte repeatSendStatus[NB_DEVICE];
byte nbTemperatureSensor = 0;
long currentMillis, oldMillis = 0;
boolean itsTimeForTemp = false;

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
	logBuildVersion();
	#endif

	// setup variable table
	buttonPins [VMC] = 0; //  BT_VMC_PIN;
	buttonPins [LIGHT] = BT_LIGHT_PIN;
	buttonPins [MIRROR] = BT_MIRROR_PIN;
	buttonPins [WC] = 0; // BT_WC_PIN;

	for (int i=0; i < NB_BUTTON; i++) {
		if (buttonPins[i] > 0) {
			pinMode(buttonPins[i], INPUT_PULLUP);
			buttonLastState[i] = digitalRead(buttonPins[i]);
			#if defined (LOG_INFO)		
			Serial.print(buttonPins[i]); Serial.print(" ");
			#endif
		}
	}
    #if defined (LOG_INFO)		
	Serial.println(" :INPUT" );
	#endif

	devicePin [VMC] = RS_VMC_PIN;
	devicePin [LIGHT] = RS_LIGHT_PIN;
	devicePin [MIRROR] = RS_MIRROR_PIN;
	devicePin [WC] = 0 ; //RS_WC_LIGHT_PIN;

	for (int theDevice=0; theDevice<NB_DEVICE; theDevice++) {
		if ( devicePin[theDevice] > 0 ) {
			pinMode(devicePin[theDevice], OUTPUT);
			deviceState[theDevice] = HIGH;
			digitalWrite(devicePin[theDevice], HIGH);
			isStatusChanged[theDevice] = true; // to not send status after each restart.
			#if defined (LOG_INFO)		
			Serial.print(String(devicePin[theDevice]) + " ");
			#endif
		}
	}
    #if defined (LOG_INFO)		
	Serial.println(" :OUTPUT" );
	#endif

	// setup rcSwitch to transmit and receive RF433
	pinMode(RFRX_PIN, INPUT);
	mySwitch.setProtocol(4);
	mySwitch.setRepeatTransmit(RADIO_REPEATS);
	mySwitch.enableReceive(RFRX_PIN-2);

	pinMode(RFTX_PIN, OUTPUT);
	mySwitch.enableTransmit(RFTX_PIN); // Transmission sur Pin

	//--------------------------------------------------------------------------------------------------------------------------------------------------
	delay(1000);

	#if (NDSPROBE > 0)
	sensors.begin();
	nbTemperatureSensor = sensors.getDeviceCount();
	#if defined(LOG_INFO)		
	Serial.println("Nb DS Sensor:"); Serial.println(nbTemperatureSensor);
	#endif
	// just to avoid probleme with some sensor where the first value returned is not correct
	for (int ploop = 0 ; ploop < NDSPROBE; ploop++) {
		getDSTemperature(ploop);
	}
	#endif
	#if (NDHT > 0)	
	dhtprobe.begin();
	#endif

	#if defined(LED_PIN)
	pinMode(LED_PIN, OUTPUT);
	blinkLed(2, 10);
	#endif

	// Start
	#if defined(LOG)		
	Serial.println("start main loop");
	#endif
	wdEnable();
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
	
	// manage temp / Humidity transmit 
	if ((millis() - tempLastCheckProbe) > (checkProbFreq / 2) ) {
		sendAllProbeValues(itsTimeForTemp);
		tempLastCheckProbe = millis();
		itsTimeForTemp = !itsTimeForTemp;
	}
	wdReset();

	// manage input button
	manageInput();
	
	wdReset();
	
	// manage RF reception
	if (mySwitch.available() && isMessage()) {
		remoteAction(RFLastSender, RFLastReceptor, RFOnOff);
	}
	wdReset();
	
	// manage status to send
	if ((millis() - tempLastSendState) > stateSendFreq) {
		sendStatus();
		//done in sendSatus tempLastSendState = currentMillis;
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
void blinkLed(int repeat, int time) {
	for (int i = 0; i < repeat; i++) {
		delay(time);
		digitalWrite(LED_PIN, HIGH);
		delay(time);
		digitalWrite(LED_PIN, LOW);
	}
}


//   _____                  _
//  |_   _|                | |
//    | | _ __  _ __  _   _| |_
//    | || '_ \| '_ \| | | | __|
//   _| || | | | |_) | |_| | |_
//   \___/_| |_| .__/ \__,_|\__|
//             | |
//             |_|
void manageInput() {
	for (int bt=0; bt < NB_BUTTON; bt++) {
		if (buttonPins[bt] > 0) {
			byte bState = digitalRead(buttonPins[bt]);
			if (bState != buttonLastState[bt]) {
				#if defined (LOG_INFO)
				Serial.print(" BUTTON ="); Serial.println( buttonPins[bt] );
				#endif
				buttonLastState[bt] = bState;
				doAction(bt, !deviceState[bt]);
				delay(DELAY_REBOUND);
			}
		}
	}
}

/*******************************************************************************
*******************************************************************************/
void remoteAction(long aSender, int aDevice, boolean aOnOff) {
	// VMC
	if (aSender == HE_IDE && aDevice < NB_DEVICE) {
		doAction(aDevice, ! aOnOff);
	} 
}

//      | |      / _ \     | | (_)
//    __| | ___ / /_\ \ ___| |_ _  ___  _ __
//   / _` |/ _ \|  _  |/ __| __| |/ _ \| '_ \
//  | (_| | (_) | | | | (__| |_| | (_) | | | |
//   \__,_|\___/\_| |_/\___|\__|_|\___/|_| |_|
//
/******
**
*******/
void doAction(byte aDevice, bool state) {
	switch (aDevice) {
	case VMC:
	case LIGHT:
	case MIRROR:
//   	case WC:
		deviceState[aDevice] = state;
		digitalWrite(devicePin [aDevice], state );
		#if defined (LOG_DEBUG)
		Serial.print("doAction #"); Serial.print(aDevice);
		Serial.print(" state:"); Serial.println(state);
		#endif		
		//@@TODO add true timer !!
		//blinkLed(3, 75); // allow arduidom to manage original RF message
		isStatusChanged[aDevice]=true;
		repeatSendStatus[aDevice]=1;
		tempLastSendState = millis();
		// RFLastAddr = 999; // reset value.
		break;

	default:
			#if defined (LOG_WARNING)
		Serial.print("doAction Unk:"); Serial.println(aDevice);
			#endif		
		break;
	}

}

/*******************************************************************************
*******************************************************************************/

bool sendStatusChanged() {
	for ( int aDevice = 0; aDevice < NB_DEVICE; aDevice++ ) {
		if (isStatusChanged[aDevice]) {
			send2RF(true , aDevice + DEVICE_STATE_ID , !deviceState[aDevice], PROBE_ID, false);
			isStatusChanged[aDevice]=false;
			#if defined (LOG_INFO)
			Serial.println("sendStatus done:");
			#endif
			
			return true; // to not send 2 status in same call
		}
	}
	return false;
}

bool sendStatusRepeat() {
	for ( int aDevice = 0; aDevice < NB_DEVICE; aDevice++ ) {
		if (repeatSendStatus[aDevice] > 0 ) {
			send2RF(true , aDevice + DEVICE_STATE_ID , !deviceState[aDevice], PROBE_ID, false);
			repeatSendStatus[aDevice]--;
			#if defined (LOG_INFO)
			Serial.println("sendStatus done (repeat):");
			#endif
			return true; // to not send 2 status in same call
		}
	}
	return false;
}

void sendStatus() {
	if ( ! sendStatusChanged() ) {
		sendStatusRepeat();
	}
	tempLastSendState = millis();
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
void sendAllProbeValues(boolean TorH) {
	sendAllProbeValues(TorH, false);
}
void sendAllProbeValues(boolean TorH, boolean checkLargeGap) {
	//@@TODO send only if change
	int probeID = TorH ? 0 : 1;

	#if defined (LOG_DEBUG)
	Serial.print("It's time to send probe, t=0 /h=1 :"); Serial.println(probeID);
	#endif
	byte totalValue = NDSPROBE + 2*NDHT;
	#if (NDHT > 0)
	int newValue = getDHT22Value(TorH);

	#if defined (LOG_DEBUG)
	Serial.print("newValue :"); Serial.println(newValue);
	Serial.print("lastValue[probeID] :"); Serial.println(lastValue[probeID]);
	#endif	

	long gap = 0;
	if ((lastValue[probeID] != newValue)) {
		gap = abs(lastValue[probeID] - newValue);
		 	#if defined (LOG_DEBUG)
		Serial.print("GapMin :"); Serial.println(gapMin[probeID]);
		Serial.print("gap :"); Serial.println(gap);
			#endif	
	}
	if ( gap >= 3*gapMin[probeID] // send temp when change >= 0.3° 
		 || (gap >= gapMin[probeID] && skeepCount[probeID] > SKEEP_MAX) // send low change 10 min 
		 || skeepCount[probeID] > 3*SKEEP_MAX
	    ) {
		sendProbeValue(probeID, newValue, true);
		lastValue[probeID] = newValue;
		skeepCount[probeID] = 0;
	} else {
		#if defined (LOG_DEBUG)
		Serial.print("No Change :"); Serial.println(TorH);
		Serial.print("skeepCount[probeID] :"); Serial.println(skeepCount[probeID]);
		#endif	
		skeepCount[probeID] += 1;
	}
	#endif

	#if (NDSPROBE > 0)
	if (tORh) {

		// @@TODO manage gap
		for ( int ploop = 2; ploop < totalValue; ploop++) {
			int tempValue = getDSTemperature(ploop);
			sendProbeValue(ploop, tempValue, true);
		}
	}
	#endif
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
	#if defined(LOG_WARNING)		
	Serial.println("DS sensor not connected");
	#endif

	if ( isnan(rawValue)  || rawValue < -10 || rawValue == 85 ) {
		return READ_ERROR;
	}
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
int getDHT22Value(boolean TorH) {
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	float rawValue = TorH ? dhtprobe.readTemperature() : dhtprobe.readHumidity();
	byte retry = 0;
	const byte RETRY_MAX = 5;

	while ( isnan(rawValue) && retry < RETRY_MAX) {
		delay(200);
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
void sendProbeValue( byte aProbeNum, int aValue, bool aSend ) {
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
		Serial.println("READ_ERROR");
		#endif
		tempValue = 0;
		decValueD = 0;
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

}



//  ______          _ _           ___  _____  _____
//  | ___ \        | (_)         /   ||____ ||____ |
//  | |_/ /__ _  __| |_  ___    / /| |    / /    / /
//  |    // _` |/ _` | |/ _ \  / /_| |    \ \    \ \
//  | |\ \ (_| | (_| | | (_) | \___  |.___/ /.___/ /
//  \_| \_\__,_|\__,_|_|\___/      |_/\____/ \____/

bool isMessage() {
	bool ret = false;
	unsigned long lRFData = mySwitch.getReceivedValue();
	unsigned long lRFAddr = mySwitch.getReceivedAddr();

	if ( RFLastSender != lRFData || RFLastAddr != lRFAddr ) {
		#if defined(LOG_INFO)	
		Serial.println(F("MySwitch available"));
		#endif
		RFLastSender = lRFData;
		RFLastAddr = lRFAddr;
		bool lRFGroup = lRFAddr > 999;
		if (lRFGroup) {
			lRFAddr -= 1000;
		}
		RFLastReceptor =  lRFAddr % 100;
		RFOnOff = (lRFAddr - RFLastReceptor) / 100;

		#if defined(LOG_DEBUG)
		Serial.print("<<  ");
		logRFMessage(RFLastSender, lRFGroup,RFLastReceptor, RFOnOff);
        #endif
		ret = true;
	} else {
		// ****** DDDDDDDEBUG ****************
		// TO BE REMOVED **********************
		// delay(10000);
	}

	mySwitch.resetAvailable();
	return ret;
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
    
    //Serial.println("FAKE mySwitch.send");
	//for (int i = 1; i <= RADIO_REPEATS; i++) {
		mySwitch.send(senderID, aFlagGroup, aReceiver, aFlagOnOff);
	//}
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

/*******************************************************************************
*******************************************************************************/
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
	Serial.println("\n start setup");
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

/* memory structure
#define MAXREMOTE 6
byte totalRegistered[NBRELAIS];
byte remoteNumber[
6 remote nbr per devive 






*/
