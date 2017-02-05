
#include "PotarMaster.h"
#include <Wire.h>


//         _____                 _                   _
//   _ _  /  __ \               | |                 | |
//  (_|_) | /  \/ ___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
//        | |    / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
//   _ _  | \__/\ (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  (_|_)  \____/\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//
//
PotarMaster::PotarMaster(byte newI2cBusID) {
	this->gI2cBusID = newI2cBusID;
	Wire.begin(); // join i2c bus (address optional for master)
	Serial.print("I2C BUS :");
	Serial.println(newI2cBusID);

}

PotarMaster::PotarMaster():PotarMaster(I2CBUSID) {
}


//         _____                _ _____         _           
//   _ _  /  ___|              | |  _  |       | |          
//  (_|_) \ `--.  ___ _ __   __| | | | |_ __ __| | ___ _ __ 
//         `--. \/ _ \ '_ \ / _` | | | | '__/ _` |/ _ \ '__|
//   _ _  /\__/ /  __/ | | | (_| \ \_/ / | | (_| |  __/ |   
//  (_|_) \____/ \___|_| |_|\__,_|\___/|_|  \__,_|\___|_|   
//                                                          
//                                                          
void PotarMaster::sendOrder(byte order) {
	
	Wire.beginTransmission(this->gI2cBusID); // transmit to device #X
	Wire.write("order is:");
	Wire.write(order);
	Wire.write(++this->orderAck);
	Wire.endTransmission();    // stop transmitting
	
	// Check ACK
	bool ackOk = false;
	String ack="Ack:";
	int a = Wire.requestFrom(int(this->gI2cBusID), int(1));    // request 6 bytes from slave device #8
  int timeout = 100;
  // wait answer
	while (Wire.available()==0 && timeout>0) {
		delay(100);
		Serial.print(".");
		timeout--;
	}
	Serial.println(Wire.available(), DEC);
  if (Wire.available() > 0 ) {
  	int c = Wire.read();
  	Serial.println(c, DEC);
  	ackOk = c == this->orderAck;
  } 
  
  // reset buffer
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    //Serial.print(c);         // print the character
  }
}

bool PotarMaster::checkAck() {
	
	// Check ACK
	bool ackOk = false;
	String ack="Ack:";
	int a = Wire.requestFrom(int(this->gI2cBusID), int(1));    // request 6 bytes from slave device #8
  int timeout = 100;
  // wait answer
	while (Wire.available()==0 && timeout>0) {
		delay(100);
		Serial.print(".");
		timeout--;
	}
	// Serial.println(Wire.available(), DEC);
  if (Wire.available() > 0 ) {
  	int c = Wire.read();
  	// Serial.println(c, DEC);
  	ackOk = c == this->orderAck;
  } 
  
  // reset buffer
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    //Serial.print(c);         // print the character
  }
  
  return ackOk;
}
