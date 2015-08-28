#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *Motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *Motor4 = AFMS.getMotor(4);

uint8_t incomingByte;
int motor1_speed;
int motor2_speed;
int motor3_speed;
int motor4_speed;

void setup() {
	pinMode(13, OUTPUT);
  Serial.begin(9600);
  delay(100);

  while (Serial.available() <= 0) {
	  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
	  delay(100);             
	  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
	  delay(100);           
  }
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz

  Motor1->setSpeed(0);
  Motor2->setSpeed(0);
  Motor3->setSpeed(0);

  Motor1->run(RELEASE);
  Motor2->run(RELEASE);
  Motor3->run(RELEASE);
}

void loop() {
  // put your main code here, t run repeatedly:
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    //Serial.print("incomingByte = ");
    //Serial.println(incomingByte,DEC);

    switch (incomingByte) {
    	case 'A': //Motor 1
    		motor1_speed = Serial.parseInt();
    		incomingByte = Serial.read(); 
  		    if (incomingByte == 'D') {
				Motor1->run(FORWARD);
			} 
			if (incomingByte == 'R') {
				Motor1->run(BACKWARD);
			}
			Motor1->setSpeed(motor1_speed);
      		Serial.print("M1 = "); Serial.print(motor1_speed, DEC); Serial.print(incomingByte); Serial.println(".");
    		break;
		case 'B': //Motor 1
    		motor2_speed = Serial.parseInt();
    		incomingByte = Serial.read(); 
    		if (incomingByte == 'D') {
				Motor2->run(FORWARD);
			} 
			if (incomingByte == 'R') {
				Motor2->run(BACKWARD);
			}
    		Motor2->setSpeed(motor2_speed);
      		Serial.print("M2 = "); Serial.print(motor2_speed, DEC); Serial.print(incomingByte); Serial.println(".");
    		break;
		case 'C': //Motor 1
    		motor3_speed = Serial.parseInt();
    		incomingByte = Serial.read(); 
      		if (incomingByte == 'D') {
				Motor3->run(FORWARD);
			} 
			if (incomingByte == 'R') {
				Motor3->run(BACKWARD);
			}
    		Motor3->setSpeed(motor3_speed);
      		Serial.print("M3 = "); Serial.print(motor3_speed, DEC); Serial.print(incomingByte); Serial.println(".");
    		break;
		case 'D': //Motor 1
    		motor4_speed = Serial.parseInt(); 
      		incomingByte = Serial.read();
      		if (incomingByte == 'D') {
				Motor4->run(FORWARD);
			} 
			if (incomingByte == 'R') {
				Motor4->run(BACKWARD);
			}
    		Motor4->setSpeed(motor4_speed);
      		Serial.print("M4 = "); Serial.print(motor4_speed, DEC); Serial.print(incomingByte); Serial.println(".");
    		break;
    	case 'S': //All Stop
			Motor1->setSpeed(0);
			Motor2->setSpeed(0);
			Motor3->setSpeed(0);
			Motor4->setSpeed(0);

			Motor1->run(RELEASE);
			Motor2->run(RELEASE);
			Motor3->run(RELEASE);
			Motor4->run(RELEASE);
	  		
	  		Serial.println("All Stop!");
			break;

		default:
			Serial.print("Command Error!!!");
			Serial.print("Get = ");
			Serial.print(incomingByte);
			Serial.println('.');

			break;		
    }

    //if (Serial.read() == '\n') {
  }



}
