


// http://www.instructables.com/id/Arduino-Push-Switch-Debouncing-Interrupts/


// Arduino Sketch par Roland Chevalier chevalir01@gmail.com
#include <EEPROM.h>
#include <Stepper.h>
#include <Wire.h>

//
#define ledPin 13

// motor
#define zeroPosPin A2 // 

#define I2CbusID 8 // I2C bus ID

#define bluIn1 9   // motor int1 blue cable
#define pinkIn2 10  // motor int2 pink cable
#define yelIn3 11   // motor int3 yellow cable
#define orgIn4 12   // motor int4 orange cable 

// const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int stepsPerRevolution = 48 * 64;
// motor cable are inverted :(
Stepper motor(stepsPerRevolution,  bluIn1,  yelIn3, pinkIn2 , orgIn4);
const int motorSpeed = 5;
const int tenSteps = (stepsPerRevolution / 64) * 10;
const int oneStep = stepsPerRevolution / 64;
const int resetStep = oneStep / 10;


// order values

const int resetPositionOrder = 100;
const int nopOrder = 0;

const int nextOneORDER = 102;
const int prevOneORDER = 101;

const int nextTenORDER = 104;
const int prevTenORDER = 103;

const int AposOrder = 110;
const int BposOrder = 120;
const int CposOrder = 130;
const int DposOrder = 140;

const int AposStep = oneStep * 9;
const int BposStep = oneStep * 20;
const int CposStep = oneStep * 30;
const int DposStep = oneStep * 42;
bool stateSwitch = false;
int currentPos = 0;
int order = resetPositionOrder;
int orderAck = 0;




/*****************************
**
** SETUP
**
******************************/
void setup() {
	Serial.begin(115200);
	
	pinMode(ledPin, OUTPUT);
	
	
    // motor
    pinMode(orgIn4, OUTPUT);
    pinMode(pinkIn2, OUTPUT);
    pinMode(yelIn3, OUTPUT);
    pinMode(bluIn1, OUTPUT);
    pinMode(zeroPosPin, INPUT_PULLUP);
  //  attachInterrupt(zeroPosPin-2,down,CHANGE); // digital pin 3 = interrup 1
    
    motor.setSpeed(motorSpeed);
    
    
	digitalWrite(ledPin, HIGH);    			
	
    down();
    
    
    Wire.begin(I2CbusID);                // join i2c bus with address #4
    Wire.onReceive(receiveEvent); // register event
    Wire.onRequest(requestEvent); // register event

    //Serial.println(" start loop");
	// order = resetPositionOrder;
	order = nextTenORDER;
	blinkLed(5, 100);

}

/*****************************                    
**
** MAIN LOOP
**
******************************/
void loop() {
    
	
	  //Wire.requestFrom(ItwoCBUSID, 6);    // request 6 bytes from slave device #8

			//stateSwitch = digitalRead(zeroPosPin);
			//Serial.println(stateSwitch);   
			
    String log="";

	delay(500);

    if (order != nopOrder) {
    	int moveValue = 0;
    	switch (order) {
    	case nopOrder :
    		break;
    		
    	case resetPositionOrder :
    		log = "resetPositionOrder";
    		resetMotorPosition();
    		break;
    		
    	case nextOneORDER :
    		log ="nextOneORDER";
    		moveValue = oneStep;
    		break;
    		
    	case prevOneORDER :
    		log ="prevOneORDER";
    		moveValue = -oneStep;
    		break;
    		
    	case nextTenORDER :
    		log ="nextTenORDER";
    		moveValue = tenSteps ;
    		break;
    		
    	case prevTenORDER :
    		log ="prevTenORDER";
    		moveValue = -tenSteps;
    		break;
    		
    	case AposOrder :
    		log ="AposOrder";
    		moveValue = AposStep - (currentPos*oneStep);
    		break;
    		
    	case BposOrder :
    		log ="AposOrder";
    		moveValue = BposStep - (currentPos*oneStep);
    		break;
    		
    	case CposOrder :
    		log ="AposOrder";
    		moveValue = CposStep - (currentPos*oneStep);
    		break;
    		
    	case DposOrder :
    		log ="AposOrder";
    		moveValue = DposStep - (currentPos*oneStep);
    		break;
    		
    	}
    	
    	if (moveValue != 0) {
    		motor.step(-moveValue);
    		currentPos += (moveValue/oneStep);
    	}
    	//order = nopOrder;
    	//Serial.println(log);
    	// Serial.print( "moveValue="); Serial.println(moveValue );
    	// Serial.print( "currentPos="); Serial.println(currentPos );
    	blinkLed(5, 100);

    }
} // EOF void loop()




/****************************
**
** Blink Led 
**
****************************/

void blinkLed(int repeat, int time) {
	for (int i = 0; i < repeat; i++) {
		delay(time);
		digitalWrite(ledPin, HIGH);
		delay(time);
		digitalWrite(ledPin, LOW);
	}
}

void down() {
	//Serial.println("-------------GOING DOWN-------------");
	stateSwitch = digitalRead(zeroPosPin);
}



// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {

  while (2 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  order = Wire.read();    // receive byte as an integer
  orderAck = Wire.read();
  //
  Serial.println(order, DEC);         // print the integer
    Serial.println(orderAck, DEC);         // print the integer

}


// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  //Wire.write("Ack:");
  Wire.write(int(orderAck)); // respond with Ack
  Serial.println(orderAck, DEC);         // print the integer

  // as expected by master
}



void resetMotorPosition() {
	for ( int ploop = 0 ; ploop < 500 && stateSwitch == 1; ploop++ ) {
		stateSwitch = digitalRead(zeroPosPin);
		
		//Serial.print("State zeroPosPin : "); log =stateSwitch);
		motor.step(resetStep);
	}
	currentPos = 0;
}


