#ifndef PotarMaster_h
#define PotarMaster_h
#include <Arduino.h>

#define I2CBUSID 8 // I2C bus ID
#define NopOrder 0
#define resetPositionOrder 100
#define prevOneORDER 101
#define nextOneORDER 102
#define prevTenORDER 103
#define nextTenORDER 104

#define AposOrder 110
#define BposOrder 120
#define CposOrder 130
#define DposOrder 140


static byte orderValues[] = {
                              NopOrder
                              , resetPositionOrder
                              , prevOneORDER, nextOneORDER, prevTenORDER, nextTenORDER
                              , AposOrder, BposOrder, CposOrder ,DposOrder
                            };

class PotarMaster {

public:
	// contructors
	PotarMaster();
	PotarMaster(byte gI2cBusID);
	//methodes
	bool sendOrder(byte order);
	bool sendOrder(byte order, byte retryNumber);

private :
	byte orderAck=0;
	byte gI2cBusID;
	bool waitBusAvailable(int);
	void clearBus();
};


#endif
