//------------------------------------------------------------------------------
//   _____       _               _   _           _      _____ _____
//  /  __ \     | |             | \ | |         | |    |  __ \  __ \
//  | /  \/ __ _| |__  _ __ __ _|  \| | ___   __| | ___| |  \/ |  \/
//  | |    / _` | '_ \| '__/ _` | . ` |/ _ \ / _` |/ _ \ | __| | __
//  | \__/\ (_| | |_) | | | (_| | |\  | (_) | (_| |  __/ |_\ \ |_\ \
//   \____/\__,_|_.__/|_|  \__,_\_| \_/\___/ \__,_|\___|\____/\____/
//
//
// Cabra Node Garage....
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
#include "PotarMaster.h"

// DEBUG LEVEL
#define LOG 1
#define LOG_DEBUG 1
#define LOG_INFO 1
// define LOG_WARNING 1

#define NDSPROBE 3 // Number of DS18B20 connected to
#define NDHT 0 // Number of DHT22 connected to
// PIN constantes
#define RFRX_PIN 2// 3 		// RF433 receiver
#define RFTX_PIN 7// 4		// RF433 transmiter
#define LED_PIN 13
const byte ONEWIRE_PIN = 9;

// Dallas DS18B20 temperature sensor bus init
#if (NDSPROBE > 0)
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire theOneWire(ONEWIRE_PIN);
DallasTemperature dallasSensors(&theOneWire);
#endif


//        _                ______          _ _
//       | |               | ___ \        | (_)
//    ___| | __ _ ___ ___  | |_/ /__ _  __| |_  ___
//   / __| |/ _` / __/ __| |    // _` |/ _` | |/ _ \
//  | (__| | (_| \__ \__ \ | |\ \ (_| | (_| | | (_) |
//   \___|_|\__,_|___/___/ \_| \_\__,_|\__,_|_|\___/
//
//
class Radio {
public:
	static const long HE_IDE = 4988678;  // default HomeEasy ID ( ID oF DIO REF:54791 ;) )
	static const byte RADIO_REPEATS = 5; // Nombre de repetitions des messages Radio
	RCSwitch mySwitch;
	// HomeEasy protocole variable
	byte RFLastReceptor = 0;
	unsigned long RFLastSender = 0; // Valeur du Data Recu par 433
	unsigned long RFLastAddr = 999; // Contains the target Id and the action to do
	// ON/OFF initialized to "impossible" value
	byte RFOnOff = 0;


	Radio::Radio() {
		mySwitch = RCSwitch();
		pinMode(RFRX_PIN, INPUT);
		mySwitch.setProtocol(4);
		mySwitch.setRepeatTransmit(RADIO_REPEATS);
		mySwitch.enableReceive(RFRX_PIN-2);

		pinMode(RFTX_PIN, OUTPUT);
		mySwitch.enableTransmit(RFTX_PIN); // Transmission sur Pin
	}

	void Radio::send2RF(boolean aFlagGroup, int aValue, int aDecValue, byte aReceiver, boolean aFlagOnOff ) {
		unsigned long senderID = 9999000 + (aValue*10)+aDecValue;
	#if defined(LOG_DEBUG)
		Serial.print("  >>");
		logRFMessage(senderID, aFlagGroup,aReceiver, aFlagOnOff);
    #endif 

		//Serial.println("FAKE mySwitch.send");
		for (int i = 1; i <= RADIO_REPEATS; i++) {
			mySwitch.send(senderID, aFlagGroup, aReceiver, aFlagOnOff);
		}

	}

	bool Radio::orderAvailable() {
		if ( this->mySwitch.available() && isMessage() ) {
		#if defined(LOG_DEBUG)
			Serial.print(F("isMessage YES")); Serial.println(RFLastSender == HE_IDE);
	    #endif 
			return RFLastSender == HE_IDE;
		}
		return false;
	}

	byte Radio::getOrder() {
		return RFLastReceptor;
	}

private:
	bool Radio::isMessage() {
		bool ret = false;
		unsigned long lRFData = this->mySwitch.getReceivedValue();
		unsigned long lRFAddr = this->mySwitch.getReceivedAddr();

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



#if defined (LOG_DEBUG)
	void logRFMessage(unsigned long aSender, byte aGroup, byte aReceptor, byte aOnOff ) {
		Serial.print(aSender);
		Serial.print(" G:");Serial.print(aGroup);
		Serial.print(" R:");Serial.print(aReceptor);
		Serial.print(" o:");Serial.println(aOnOff);
	}
#endif

};


class Probe {
	static const unsigned int READ_ERROR = 9999;
	//static byte nbDSProbeCount;
	static const byte SKEEP_MAX = 3;
	//static DallasTemperature *ptrSensor;
public:
	// static DallasTemperature *ptrSensor;
	static const byte DS18B20 = 0;
	static const byte DHT_H = 1;
	static const byte DHT_T = 2;

	int gapMin =0;
	byte ID = 0;
	byte index = 0;
	int lastValue = 9000;
	byte skeepCount = 0;
	byte skeepMax = SKEEP_MAX;

	Probe::Probe(byte newType, byte newID, byte newIndex){
		type = newType;
		ID = newID;
		index = newIndex;
		if (newType == DHT_H) {
			gapMin = 100; // 1%
		} else {
			gapMin = 10; // 0.1 °C
		}
		//Probe::nbDSProbeCount++;
	}
	Probe::Probe(byte newType, byte newID):Probe(newType, newID,0){
	}

	Probe::Probe():Probe(DS18B20, 0, 0){
	}
	/**
	*/
	byte Probe::getType(){
		return this->type;
	}
	/**
	*/
	int Probe::readValue() {
		// *** MORE THAN ONE RETURN ****
		float rawValue = NAN;
		dallasSensors.requestTemperatures();
		rawValue = dallasSensors.getTempCByIndex(this->index);

		if ( isnan(rawValue) || rawValue < -10 || rawValue == 85 || rawValue > 125 ) {
		#if defined(LOG_WARNING)		
			Serial.println("DS #") ; Serial.print(this->index) ;
			Serial.println(" off or error");
		#endif
			return READ_ERROR;
		}
	#if defined(LOG_DEBUG)		
		Serial.print("DS #") ; Serial.print(this->index);
		Serial.print(" t="); Serial.println(rawValue);
	#endif
		return rawValue*100;
	}
	
	
	/***************************************************************************
	*/
	bool checkNewValue() {
	#if defined (LOG_DEBUG)
		Serial.print("sendProbeValues #"); Serial.print(this->index);
		Serial.print(" Type="); Serial.println(this->getType());
	#endif
		int newValue = this->readValue();
		// @@RC newValue = 100+10*flaseValue++;

	#if defined (LOG_DEBUG)
		Serial.print("newValue="); Serial.print(newValue);
		Serial.print(" lastValue="); Serial.println(this->lastValue);
	#endif	

		long gap = 0;
		if ((this->lastValue != newValue)) {
			gap = abs(this->lastValue - newValue);
		#if defined (LOG_DEBUG)
			Serial.print("GapMin="); Serial.print(this->gapMin);
			Serial.print(" gap="); Serial.println(gap);
		#endif	
		}
		if ( gap >= 3*this->gapMin // send temp when change >= 0.3°
		        || (gap >= this->gapMin && this->skeepCount > (SKEEP_MAX/2) ) // send low change every 10 min if any (depend of conf)
		        || this->skeepCount > SKEEP_MAX // send value every 20 min ( depend of conf )
		   ) {
		//@@ICI
			// this->lastValue = sendProbeValue(lProbeNum, newValue, true);
			this->lastValue = newValue;
			Serial.print(" Need to send ="); Serial.println(this->lastValue );
			this->skeepCount = 0;
			return true;
		} else {
		#if defined (LOG_DEBUG)
			Serial.print("No Change / skeepCount="); Serial.println(this->skeepCount);
		#endif	
			this->skeepCount++;
			return false;
		}

	}
	
	String Probe::toString() {
		String thisStr = "Probe::ID:"; thisStr += this->ID;
		thisStr += " type:"; thisStr += this->type; 
		thisStr += " gapMin:"; thisStr += this->gapMin; 
		thisStr += " lastValue:"; thisStr += this->lastValue; 
		return thisStr;
	}


private:
	byte type;

};


// GLOBAL constantes

const unsigned int READ_ERROR = 9999;

const byte PROBE_ID = 8; // ID of probe beetween 0 and 15
// const byte DEVICE_STATE_ID = 5; // first state identifier added to device number


// #define DHT_PIN 9           // DTH 22 Probe

Radio theRadio = Radio();

Probe* ptrProbes[NDSPROBE + NDHT*2]; // same value for all


// minimum gap to send value 10 for 0.1°C, and 100 for 1%

// DHT initialisation
#if (NDHT > 0)
#include "DHT.h"
DHT dhtprobe(DHT_PIN, DHT22);
#endif



PotarMaster potar = PotarMaster();
unsigned long tempLastCheckProbe = 0 ;

//unsigned long checkProbFreq = 5*60000; //5 * 1 minutes
unsigned long checkProbFreq = 50000; //10 sec

const byte SKEEP_MAX = 3;

// Command group
// 0 = inputPin, 1
byte 	nbDSProbeCount = 0;
long 	currentMillis, oldMillis = 0;
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
	#if defined(LED_PIN)
	pinMode(LED_PIN, OUTPUT);
	blinkLed(2, 10);
	#endif
	wdDisable();
	#if defined(LOG)	
	Serial.begin(115200); // Init du Port serie/USB
	logBuildVersion();
	#endif


    #if defined (LOG_INFO)		
	Serial.println(" :OUTPUT" );
	#endif

	//--------------------------------------------------------------------------------------------------------------------------------------------------
	delay(1000);

	#if (NDSPROBE > 0)
	dallasSensors.begin();
	nbDSProbeCount = dallasSensors.getDeviceCount();  // @@RCSIMU nbDSProbeCount = 3;
	#if defined(LOG_INFO)		
	Serial.println("Nb DS Sensor:"); Serial.println(nbDSProbeCount);
	#endif
	// just to avoid probleme with some sensor where the first value returned is not correct
	for (int indexprobe = 0 ; indexprobe < NDSPROBE; indexprobe++) {
		//getDSTemperature(indexprobe);
		ptrProbes[indexprobe] = new Probe (Probe::DS18B20, PROBE_ID+indexprobe , indexprobe);
		ptrProbes[indexprobe]->readValue();
		Serial.println(ptrProbes[indexprobe]->toString());
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
		sendAllProbeValues(nbDSProbeCount + NDHT*2);
		tempLastCheckProbe = millis();
		itsTimeForTemp = !itsTimeForTemp;
	}
	wdReset();
	// manage RF reception
	if ( theRadio.orderAvailable() ) {
		
		wdDisable();
		doAction(theRadio.getOrder());
		wdEnable();
	}
	wdReset();
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



//      | |      / _ \     | | (_)
//    __| | ___ / /_\ \ ___| |_ _  ___  _ __
//   / _` |/ _ \|  _  |/ __| __| |/ _ \| '_ \
//  | (_| | (_) | | | | (__| |_| | (_) | | | |
//   \__,_|\___/\_| |_/\___|\__|_|\___/|_| |_|
//
/******
**
*******/
void doAction(byte order) {

														#if defined(LOG_DEBUG)
	Serial.print("motor>>");Serial.println(order);
														#endif

	if (order < 10 ) {
		bool ok = false;
		int timeout=0;
		while ( !ok ) {
			potar.sendOrder(orderValues[order]);
			ok = potar.checkAck() || ++timeout>2;
		}
	}
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
	Probe* ptrProbe;
	for ( int numProbe = 0;  numProbe <nbTotalProbe ; numProbe++) {
		ptrProbe = ptrProbes[numProbe];
		bool needToSend = ptrProbe->checkNewValue();
		if ( needToSend ) {
															#if defined(LOG_INFO)		
			Serial.println(ptrProbe->toString());
															#endif
			sendProbeValue(ptrProbe, true);	
		}
	}
}


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
int sendProbeValue( Probe* theProbe, bool aSend ) {
	int decValueD;
	int aValue = theProbe->lastValue;
	bool positive = aValue > 0;
	int tempValue;
	#if defined(LOG_INFO)	
	Serial.print(">>Probe:" ); Serial.print(theProbe->ID);
	Serial.print(" v:"); Serial.println(aValue, DEC);
	#endif

	// bad values returned by the sensor >-10 or 85
	if ( aValue == READ_ERROR ) {
		#if defined(LOG_INFO)		
		#endif
		if ( aValue == 9000 ) { // Bizard ?????????
			Serial.println("READ_ERROR 99");
			tempValue = 95;
		} else {
			Serial.print("READ_ERROR 98"); Serial.println(aValue);
			tempValue = 90;
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
	Serial.print(" ID: " ); Serial.print( theProbe->ID, DEC);
	Serial.print(" value: " );
	Serial.print(tempValue); Serial.print("."); Serial.print(decValueD);
	Serial.print(" Signe: " ); Serial.println( positive, DEC);
#endif
	if ( aSend ) {
		theRadio.send2RF(false , tempValue, decValueD, theProbe->ID, positive);
	}
	return tempValue*100;
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
/*void logRFMessage(unsigned long aSender, byte aGroup, byte aReceptor, byte aOnOff ) {
	Serial.print(aSender);
	Serial.print(" G:");Serial.print(aGroup);
	Serial.print(" R:");Serial.print(aReceptor);
	Serial.print(" o:");Serial.println(aOnOff);
}*/
#endif


