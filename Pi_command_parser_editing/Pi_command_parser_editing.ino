



// Purpose : Pi command parser
// Author : Marco Sin
// Made on : March 8th, 2019
// --------------------------------------

// IMPORTANT NOTE
// SINCE RX AND TX ARE USED TO PROGRAM THE ARDUINO, IF THE PROGRAM CRASHES IT MAY NOT RESPOND AND THUS BE INCAPABLE OF BEING REPORGRAMMED.
// TO FIX THIS YOU NEED TO PRESS THE RESET BUTTON AND QUICK UPLOAD A PROGRAM OF ANY TYPE (PREFERABLY A BLANK ONE)

#include <Sabertooth.h>
#include <SoftwareSerial.h>
#include <Cytron_SmartDriveDuo.h>
// ----- Pins ------

/* 
Note from Marco: The pin configuration is setup for the Arduino Uno
1 = ON .  0 = OFF
Configuration notes:
On the Sabertooth controllers:  The Arduino should be connected to the S1 signals pins of the motor controllers
-----------------------------------------------------------------------------------------
The motor controller responsible for "Driving" should be configured as such:
000111
-----------------------------------------------------------------------------------------
The motor controller responsible for the Feed mechanism should be configured as such:
000011
-----------------------------------------------------------------------------------------
CYtron controller or shooting motor: The arduino needs to be connecting to the "IN1" pin
111000
-----------------------------------------------------------------------------------------
 */

// input command pins.     *** THESE PINS ARE DIGITAL
int inprightMotor = 6;  // Connected to pin 12 on Pi
int inpleftMotor = 10;  // connected to pin 11 on Pi
int inpshootMotor = 3;  // Connected to ping 13 on Pi

// output command pins *** IT IS VERY IMPORTANT THESE PINS ARE PWM CAPABLE ***
// RX (0) and TX(1) pins are reserverd for DriverMotor and FeedMotor ONLY
// Feed motor controller needs to be connected to pin 11
// Drive motor connected to pin 1 or TX

// Debug code
int rightLed = 2;  // RED LED
int leftLed = 4;   // YELLOW LED
int shootLed = 7;  // GREEN LED
// \ Debug

// ----------

// Initialization
Sabertooth STDrive(128); // Declaring Sabertooth Drive Motor Controller address
Sabertooth STFeed(129); //  Declaring Sabertooth Feed Motor controller address

#define IN1 8 // Arduino pin 8is connected to MDDS30 pin IN1.
#define AN1 9 // Arduino pin 9 is connected to MDDS30 pin AN1.
#define AN2 12 // Arduino pin 12 is connected to MDDS30 pin AN2.
#define IN2 13 // Arduino pin 13 is connected to MDDS30 pin IN2.
Cytron_SmartDriveDuo smartDriveDuo30(PWM_INDEPENDENT, IN1, IN2, AN1, AN2);
// For CYtron motors speed is governed by a signed int between -1 and 1.  1 is forward and -1 is backwards 0 is stop
signed int speedLeft, speedRight;

void setup() {
  pinMode(inpleftMotor, INPUT);  // These two pins control direction in form of left:right.  
  pinMode(inprightMotor, INPUT); // 10 = left; 01 = right; 00 = Off 
  pinMode(inpshootMotor, INPUT); // 1 = On; 0 = off
  attachInterrupt(digitalPinToInterrupt(inpshootMotor), shoot, RISING);

  pinMode(rightLed, OUTPUT);
  pinMode(leftLed, OUTPUT);
  pinMode(shootLed, OUTPUT);
  digitalWrite(rightLed, HIGH);
  SabertoothTXPinSerial.begin(9600);
  


}

void loop() {
  // Drive controls
  int right = digitalRead(inprightMotor);
  int left = digitalRead(inpleftMotor);
  int shoot = digitalRead(inpshootMotor);
  
  
  if ((right == 0) and (left == 0)) { // Stop turning
    digitalWrite(rightLed, LOW);  // debug
    digitalWrite(leftLed, LOW);   // debug
    STDrive.motor(1, 0);
    STDrive.motor(2, 0);
  } 
  if ((right == 1) and (left == 0)) { // Turn right
    digitalWrite(rightLed, HIGH); // debug
    digitalWrite(leftLed, LOW);   // debug
    STDrive.motor(1, 127);
    STDrive.motor(2, -127);
  }
  if ((right == 0) and (left > 0)) { // Turn left
    digitalWrite(rightLed, LOW); // debug
    digitalWrite(leftLed, HIGH); // debug
    STDrive.motor(1, -127);
    STDrive.motor(2, 127);
  } 

  

}

void shoot() {
    int powerm;
    digitalWrite(shootLed, HIGH); // debug
    digitalWrite(rightLed, LOW);  // debug
    digitalWrite(leftLed, LOW);   // debug
    STDrive.motor(1, 0);
    STDrive.motor(2, 0);
    for (powerm = 0; powerm <30; powerm ++) { // ramps up shoot motor slowly to not burn out fuse
      smartDriveDuo30.control(powerm, powerm);
      digitalWrite(shootLed, LOW); // debug
      delay(40);
      digitalWrite(shootLed, HIGH); // debug
    }
    STFeed.motor(1, 127); // Feed a ball (or two)
    STFeed.motor(2, 127);
    delay(300);  // Stop feeding and shooting motors
    STFeed.motor(1, 0);
    STFeed.motor(2, 0);
    smartDriveDuo30.control(0, 0);
    digitalWrite(shootLed, LOW); // debug
  }
