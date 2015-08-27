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

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz

  Motor1->setSpeed(0);
  Motor2->setSpeed(0);
  Motor3->setSpeed(0);
  Motor4->setSpeed(0);

  Motor1->run(RELEASE);
  Motor2->run(RELEASE);
  Motor3->run(RELEASE);
  Motor4->run(RELEASE);
}

void loop() {
  // put your main code here, to run repeatedly:

  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    //incomingByte = Serial.read();

    int motor1_speed = Serial.parseInt(); 
    int motor2_speed = Serial.parseInt(); 
    int motor3_speed = Serial.parseInt(); 
    int motor4_speed = Serial.parseInt(); 
    
    // say what you got:
    //Serial.print("I received: ");
    //Serial.println(incomingByte, DEC);

    incomingByte = Serial.read();
    Serial.print("incomingByte = ");
    Serial.println(incomingByte,DEC);


    //if (Serial.read() == '\n') {
    if (incomingByte == 'R') {
      Motor1->run(FORWARD);
      Motor2->run(FORWARD);
      Motor3->run(FORWARD);
      Motor4->run(FORWARD);

      Motor1->setSpeed(motor1_speed);
      Motor2->setSpeed(motor2_speed);
      Motor3->setSpeed(motor3_speed);
      Motor4->setSpeed(motor4_speed);

      Serial.print("M1 = "); Serial.print(motor1_speed, DEC); Serial.println(".");
      Serial.print("M2 = "); Serial.print(motor2_speed, DEC); Serial.println(".");
      Serial.print("M3 = "); Serial.print(motor3_speed, DEC); Serial.println(".");
      Serial.print("M4 = "); Serial.print(motor4_speed, DEC); Serial.println(".");
      Serial.println(" ");
    }
  }



}
