//------------------------------------------------------------------------------
//   _____       _               _   _           _      _____ _____ ___  ___
//  /  __ \     | |             | \ | |         | |    |  __ \  __ \|  \/  |
//  | /  \/ __ _| |__  _ __ __ _|  \| | ___   __| | ___| |  \/ |  \/| .  . |
//  | |    / _` | '_ \| '__/ _` | . ` |/ _ \ / _` |/ _ \ | __| | __ | |\/| |
//  | \__/\ (_| | |_) | | | (_| | |\  | (_) | (_| |  __/ |_\ \ |_\ \| |  | |
//   \____/\__,_|_.__/|_|  \__,_\_| \_/\___/ \__,_|\___|\____/\____/\_|  |_/
//                                                                          
// Cabra Node Garage et Meteo....
//
// Author  : chevalir
// version : 2.0
// Ajout une DS pour temperature chaudiere
//------------------------------------------------------------------------------
/*
 http://patorjk.com/software/taag/#p=display&v=3&c=c%2B%2B&f=Doom&t=TypeHere
*/
//------------------------------------------------------------------------------
#include <Arduino.h>
//#include <avr/wdt.h> // watchDog API

////////////////////////////////////////////////////////////////////////////////
#include "Arduidom_Radio.h"

// DEBUG LEVEL
#define LOG 1
//#define LOG_DEBUG 1
#define LOG_INFO 1
// define LOG_WARNING 1

#define NDSPROBE 2 // Number of DS18B20 connected to
#define NDHT 1 // Number of DHT22 connected to
// PIN constantes
#define RFRX_PIN 2   // RF433 receiver
#define RFTX_PIN 7   // RF433 transmiter
#define LED_PIN 13
#define ONEWIRE_PIN 9 // @@RC
#define DHT_PIN 10    // DTH 22 Probe @@RC

#ifdef POTAR
#include "PotarMaster.h"
PotarMaster potar = PotarMaster();
#endif


#define NBMP085 1  // I2C address of BMP085

#ifdef NBMP085
#include <BMP085.h>
BMP085 bmp;
#endif


// Dallas DS18B20 temperature sensor bus init
#if (NDSPROBE > 0)
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire theOneWire(ONEWIRE_PIN);
DallasTemperature dallasSensors(&theOneWire);
#endif

// DHT initialisation
#if (NDHT > 0)
#include "DHT.h"
DHT dhtprobe(DHT_PIN, DHT22);
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
		mySwitch.setProtocol(4);		
	#if defined(RFRX_PIN)		
		pinMode(RFRX_PIN, INPUT);
		mySwitch.enableReceive(RFRX_PIN-2);
    #endif
	#if defined(RFTX_PIN)		    
		mySwitch.setRepeatTransmit(RADIO_REPEATS);
		pinMode(RFTX_PIN, OUTPUT);
		mySwitch.enableTransmit(RFTX_PIN); // Transmission sur Pin
    #endif		
	}

	void Radio::send2RF(boolean aFlagGroup, int aValue, int aDecValue, byte aReceiver, boolean aFlagOnOff ) {
		unsigned long senderID = 9999000 + (aValue*10)+aDecValue;
	#if defined(LOG_INFO)
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

		#if defined(LOG_INFO)
			Serial.print("<<  ");
			logRFMessage(RFLastSender, lRFGroup,RFLastReceptor, RFOnOff);
        #endif
			ret = true;
		} 
		mySwitch.resetAvailable();
		return ret;
	}



#if defined (LOG_INFO)
	void logRFMessage(unsigned long aSender, byte aGroup, byte aReceptor, byte aOnOff ) {
		Serial.print(aSender);
		Serial.print(" G:");Serial.print(aGroup);
		Serial.print(" R:");Serial.print(aReceptor);
		Serial.print(" o:");Serial.println(aOnOff);
	}
#endif

};


class Probe {
	static const byte SKEEP_MAX = 3;
public:
	static const unsigned int READ_ERROR = 9999;
	static const byte TypeDS = 0;
	static const byte TypeDHTH = 1;
	static const byte TypeDHTT = 2;
	static const byte TypeBMP085 = 3;

	int gapMin =0;
	byte ID = 0;
	byte index = 0;
	int lastValue = 9000;
	byte skeepCount = 0;
	byte skeepMax = SKEEP_MAX;

	Probe::Probe(byte newType, byte newID, byte newIndex, byte newGape){
		type = newType;
		ID = newID;
		index = newIndex;
		gapMin = newGape;
	}
	/** GET TYPE
	*/
	byte Probe::getType(){
		return this->type;
	}
	
	/**  readValue ()
	    must be define by each real probe. 
	*/
	virtual int Probe::readValue()=0;

	/*************************************************************************
	* read the new value in probe 
	* return true if the diff with the last value is > to the gap
	*/
	bool checkNewValue() {
	    #if defined (LOG_INFO)
		Serial.print("\n checkNewValue #"); Serial.println(this->index);
		//Serial.print(" Type="); Serial.println(this->getType());
	    #endif
		int newValue = readValue();
	    #if defined (LOG_INFO)
		Serial.print("newV="); Serial.print(newValue);
		Serial.print(" lastV="); Serial.println(this->lastValue);
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
			this->lastValue = newValue;
			Serial.print("Need2send="); Serial.println(this->lastValue );
			this->skeepCount = 0;
			// more than one return
			return true;
		} else {
		#if defined (LOG_DEBUG)
			Serial.print("No Change Time="); Serial.println(this->skeepCount);
		#endif	
			this->skeepCount++;
			// more than one return			
			return false;
		}

	}

	String Probe::toString() {
		String thisStr = "Probe::ID:"; thisStr += this->ID;
		thisStr += " type:"; thisStr += this->type;
		thisStr += " index:"; thisStr += this->index;
		thisStr += " gapMin:"; thisStr += this->gapMin;
		thisStr += " lastValue:"; thisStr += this->lastValue;
		return thisStr;
	}


private:
	byte type;

};
#if (NDSPROBE > 0)
class ProbeDS : public Probe  {
	
public:
	ProbeDS::ProbeDS(byte newID, byte newIndex): Probe(Probe::TypeDS,  newID,  newIndex, 10){
	}	
	
	int ProbeDS::readValue() {
	#if defined (LOG_DEBUG)
		Serial.println("DS::readValue()");
	#endif
		
		// *** MORE THAN ONE RETURN ****
		float rawValue = NAN;
		dallasSensors.requestTemperatures();
		rawValue = dallasSensors.getTempCByIndex(this->index);
		if ( isnan(rawValue) || rawValue < -55 || rawValue == 85 || rawValue > 125 ) {
		#if defined(LOG_WARNING)		
			Serial.print("DS#") ; Serial.print(this->index) ; Serial.println(" off or err");
		#endif
			return Probe::READ_ERROR;
		}		
	#if defined(LOG_INFO)		
		Serial.print("DS#") ; Serial.print(this->index);
		Serial.print(" t="); Serial.println(rawValue);
	#endif
		return rawValue*100;
	}
	
};
#endif
#if (NDHT > 0)
class ProbeDHTT : public Probe  {
	
public:
	ProbeDHTT::ProbeDHTT(byte newID, byte newIndex):Probe(Probe::TypeDHTT,  newID,  newIndex , 10){
	}	
		
	int ProbeDHTT::readValue() {
		// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	#if defined (LOG_DEBUG)
		Serial.println("DHTT::readValue()");
	#endif
		
		float rawValue = dhtprobe.readTemperature(); 
		byte retry = 0;
		const byte RETRY_MAX = 10;
	
		while ( isnan(rawValue) && retry < RETRY_MAX) {		
			delay(100);
			rawValue = dhtprobe.readTemperature(); 
			retry++;
		}
		#if defined(LOG_INFO)		
		Serial.print("DHT T="); Serial.println(rawValue);
		#endif
	
		if (isnan(rawValue)) {
			return Probe::READ_ERROR;
		}
		return rawValue*100;
	}
};

class ProbeDHTH : public Probe  {
public:
	ProbeDHTH::ProbeDHTH(byte newID, byte newIndex): Probe(Probe::TypeDHTH,  newID,  newIndex, 100){}	
		
	int ProbeDHTH::readValue() {
	#if defined (LOG_DEBUG)
		Serial.println("ProbeDHTH::readValue");
	#endif
		
		// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
		float rawValue = dhtprobe.readHumidity();
		byte retry = 0;
		const byte RETRY_MAX = 10;
	
		while ( isnan(rawValue) && retry < RETRY_MAX) {		
			delay(100);
			rawValue = dhtprobe.readHumidity();
			retry++;
		}
		#if defined(LOG_INFO)		
		Serial.print("DTH H="); Serial.println(rawValue);
		#endif
	
		if (isnan(rawValue)) {
			return Probe::READ_ERROR;
		}
		return rawValue*100;
	}
};
#endif

class ProbeBMP085 : public Probe  {
public:
	ProbeBMP085::ProbeBMP085(byte newID, byte newIndex): Probe(Probe::TypeBMP085,  newID,  newIndex, 50){	}
	
	/* READVALUE */
	int ProbeBMP085::readValue() {
		#if defined (LOG_DEBUG)
		Serial.println("BMP085::readValue");
		#endif
		long seaLevelPressure = bmp.getSeaLevel(bmp.readFloatPressure(), 235);
		#if defined(LOG_INFO)		
		Serial.print("seaPr=");
		Serial.println(seaLevelPressure/100);
		#endif
  		
		return seaLevelPressure - 100000; // 
	}
		
};

class ProbeBMP085T : public Probe  {
public:
	ProbeBMP085T::ProbeBMP085T(byte newID, byte newIndex): Probe(Probe::TypeBMP085,  newID,  newIndex, 10){	}
	
	/* READVALUE */
	int ProbeBMP085T::readValue() {
		#if defined (LOG_DEBUG)
		Serial.println("BMP085::readValue");
		#endif
        double realTemperature = bmp.readTemperature();		
		#if defined(LOG_INFO)		
		Serial.print("T=");
		Serial.println(realTemperature);
		#endif
  		realTemperature *=100;
		return realTemperature;
	}
		
};



//*$****************************************************************************
//   _____ _       _           _   _   _       _                 
//  |  __ \ |     | |         | | | | | |     | |                
//  | |  \/ | ___ | |__   __ _| | | | | | __ _| |_   _  ___  ___ 
//  | | __| |/ _ \| '_ \ / _` | | | | | |/ _` | | | | |/ _ \/ __|
//  | |_\ \ | (_) | |_) | (_| | | \ \_/ / (_| | | |_| |  __/\__ \
//   \____/_|\___/|_.__/ \__,_|_|  \___/ \__,_|_|\__,_|\___||___/
//                                                               
//     

const byte PROBE_ID = 8; // ID from 28 To ....


Radio* theRadio;

Probe* ptrProbes[NDSPROBE + NDHT*2 + NBMP085*2]; // same value for all


// minimum gap to send value 10 for 0.1°C, and 100 for 1%


unsigned long tempLastCheckProbe = 0 ;

unsigned long checkProbFreq = 10*60000; //10 * 1 minutes
//unsigned long checkProbFreq = 30000;  //30 sec

const byte SKEEP_MAX = 5;

// Command group
// 0 = inputPin, 1
byte nbTotalProbe = 0;

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
	#if defined(LED_PIN)
	pinMode(LED_PIN, OUTPUT);
	blinkLed(2, 10);
	#endif
	#if defined(LOG)	
	Serial.begin(115200); // Init du Port serie/USB
	logBuildVersion();
	#endif

	//--------------------------------------------------------------------------------------------------------------------------------------------------
	delay(1000);
    theRadio = new Radio();	
	#if (NDHT > 0)	// manage only one DHT
	dhtprobe.begin();
	ptrProbes[nbTotalProbe] = new ProbeDHTT (PROBE_ID+nbTotalProbe , nbTotalProbe);
	ptrProbes[nbTotalProbe]->readValue();
	Serial.println(ptrProbes[nbTotalProbe]->toString());
	nbTotalProbe++;
	ptrProbes[nbTotalProbe] = new ProbeDHTH (PROBE_ID+nbTotalProbe , nbTotalProbe);
	ptrProbes[nbTotalProbe]->readValue();
	Serial.println(ptrProbes[nbTotalProbe]->toString());
	nbTotalProbe++;
	#endif

	#if (NBMP085 > 0)
	InitializeBMP085();
	ptrProbes[nbTotalProbe] = new ProbeBMP085T (PROBE_ID+nbTotalProbe , nbTotalProbe);
	ptrProbes[nbTotalProbe]->readValue();
	Serial.println(ptrProbes[nbTotalProbe]->toString());
	nbTotalProbe++;
	ptrProbes[nbTotalProbe] = new ProbeBMP085 (PROBE_ID+nbTotalProbe , nbTotalProbe);
	ptrProbes[nbTotalProbe]->readValue();
	Serial.println(ptrProbes[nbTotalProbe]->toString());
	nbTotalProbe++;	
	#endif
	
	#if (NDSPROBE > 0)
	dallasSensors.begin();
	int nbDSProbeCount = dallasSensors.getDeviceCount();  // @@RCSIMU nbDSProbeCount = 3;
	#if defined(LOG_DEBUG)		
	Serial.println("Nb DS:"); Serial.println(nbDSProbeCount);
	#endif
	// just to avoid probleme with some sensor where the first value returned is not correct
	int indexDS;
	for (indexDS = nbTotalProbe ; indexDS < nbTotalProbe+NDSPROBE; indexDS++) {
		ptrProbes[indexDS] = new ProbeDS (PROBE_ID+indexDS , indexDS-nbTotalProbe);
		ptrProbes[indexDS]->readValue();
		Serial.println(ptrProbes[indexDS]->toString());
	}
	nbTotalProbe += NDSPROBE;	
	#endif
	
	#if defined(LED_PIN)
	pinMode(LED_PIN, OUTPUT);
	blinkLed(2, 10);
	#endif

	// Start
	#if defined(LOG)
	Serial.print("Start loop ProbesNb="); Serial.println(nbTotalProbe);
	#endif
	
	tempLastCheckProbe = millis() - checkProbFreq + 10000;

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
	// manage all probes data transmit
	if ((millis() - tempLastCheckProbe) > checkProbFreq ) {
		sendAllProbes(nbTotalProbe);
		tempLastCheckProbe = millis();
	} else {
	#if defined (LOG_DEBUG)
		Serial.print("elpase time:");
		Serial.print((millis() - tempLastCheckProbe) / 1000 );
		Serial.print("/"); Serial.println(checkProbFreq / 1000);
		delay(10000);
	#endif
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


//   _____                _  ___  _ _______          _               
//  /  ___|              | |/ _ \| | | ___ \        | |              
//  \ `--.  ___ _ __   __| / /_\ \ | | |_/ / __ ___ | |__   ___  ___ 
//   `--. \/ _ \ '_ \ / _` |  _  | | |  __/ '__/ _ \| '_ \ / _ \/ __|
//  /\__/ /  __/ | | | (_| | | | | | | |  | | | (_) | |_) |  __/\__ \
//  \____/ \___|_| |_|\__,_\_| |_/_|_\_|  |_|  \___/|_.__/ \___||___/
//                                                                   
//                                                                   
void sendAllProbes(byte nbProbe) {
	Probe* ptrProbe;
	#if defined(LOG_INFO)		
	Serial.println("start sendAll...");
	#endif

	for ( int numProbe = 0;  numProbe <nbProbe ; numProbe++) {
		ptrProbe = ptrProbes[numProbe];
		//if (ptrProbe->getType() == Probe::TypeBMP085) {
			bool needToSend = ptrProbe->checkNewValue();
			if ( needToSend ) {
				#if defined(LOG_INFO)		
				Serial.println(ptrProbe->toString());
				#endif
				sendProbeValue(ptrProbe, true);
				delay(5000);			
			} 
		//}
	}

	#if defined(LOG_DEBUG)		
	Serial.println("end sendAll...");
	#endif

}

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
	if ( ! positive ) aValue = 0 - aValue;
	
	
	int tempValue;
	#if defined(LOG_INFO)	
	Serial.print(">>Probe:" ); Serial.print(theProbe->ID);
	Serial.print(" v:"); Serial.println(aValue, DEC);
	#endif

	// bad values returned by the sensor >-10 or 85
	if ( aValue == Probe::READ_ERROR ) {
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
	Serial.print(" (Pin " ); Serial.print( theProbe->ID+20, DEC);	
	Serial.print(") value: " );
	Serial.print(tempValue); Serial.print("."); Serial.print(decValueD);
	Serial.print(" Signe: " ); Serial.println( positive, DEC);
#endif
	if ( aSend ) {
		//Serial.print(" FACK SEND " ); // 
		theRadio->send2RF(false , tempValue, decValueD, theProbe->ID, positive);
	}
	return tempValue*100;
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
#endif




//  _________  _________   _____ _____  _____ 
//  | ___ \  \/  || ___ \ |  _  |  _  ||  ___|
//  | |_/ / .  . || |_/ / | |/' |\ V / |___ \ 
//  | ___ \ |\/| ||  __/  |  /| |/ _ \     \ \
//  | |_/ / |  | || |     \ |_/ / |_| |/\__/ /
//  \____/\_|  |_/\_|      \___/\_____/\____/ 
//                                            
//   


#if defined (LOG)
void checkSettingsBMP085()
{
	Serial.print("BMP085 v:");
	long bmpVersion = bmp.getVersion();
	Serial.print(bmpVersion >> 8); Serial.print("."); Serial.print(bmpVersion & 0xFF);
	Serial.print(" (0x"); Serial.print(bmpVersion, HEX); Serial.print(") oversampling:");
	// Serial.print("Oversampling: ");
	Serial.println(bmp.getOversampling());
	//Software Oversampling: 
	Serial.print("over:");
	Serial.println(bmp.getSoftwareOversampling());
}
#endif

void InitializeBMP085() {
	  // Initialize BMP085 or BMP180 sensor
#if defined(LOG_INFO)		
  Serial.println("InitializeBMP085");
#endif

  // Ultra high resolution: BMP085_ULTRA_HIGH_RES
  // (default) High resolution: BMP085_HIGH_RES
  // Standard: BMP085_STANDARD
  // Ultra low power: BMP085_ULTRA_LOW_POWER
  int timeout = 0;
  while(!bmp.begin(BMP085_ULTRA_HIGH_RES) && (timeout<10) )
  {
    Serial.println("init BMP085 error");
    timeout += 1;
    delay(500);
  }

  // Enable or disable SOSS (Software oversampling)- Use with BMP085_ULTRA_HIGH_RES !
  // For applications where a low noise level is critical, averaging is recommended if the lower bandwidth is acceptable
  // Conversion time pressure: 76.5ms, RMS noise 0.02 hPA / 0.17 m
  bmp.setSoftwareOversampling(0);

  // Check settings
  checkSettingsBMP085();
}


