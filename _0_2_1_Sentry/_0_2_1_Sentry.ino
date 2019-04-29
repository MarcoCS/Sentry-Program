// Created By Marco Sin
// Purpose of program: Control Robot
// Ver. 0.4.1 (As of April 26)
/*
Program receives incoming float values over serial.  Then handles
motor controls and uses a gyroscope to (hopefully!) precisely aim and shoot at
stationary targets
*/


/*
Added: multi-data type exception to function "printf" Ver. 0.2.0

Bugfixes: Added a garbage value to be displayed when no targets are present. ver. 0.2.1
*/
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <LCD.h>
#include <MPU6050_tockn.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
MPU6050 mpu6050(Wire);

// Variables
float targetAngle;
float parsedAngle;

// Pin(s)
const int Ready = 12;

// Servos
Servo Drive; // Drive motors (duh!)
Servo Feed;  // Motor which loads ball into launcher


void setup() {
  // Setting some stuff up
  Serial.begin(115200);
  lcd.begin(16,2);
  pinMode(Ready, OUTPUT);
  Wire.begin();
  mpu6050.begin();
  digitalWrite(Ready, HIGH);
  Drive.attach(9);
  Feed.attach(5);

  // Issuing stop commands to both motors
  Drive.writeMicroseconds(1500); 
  Feed.writeMicroseconds(1500);  

  // Just notifying user when program is ready!
  printf("Calibrating gyro", 0, 0);
  printf("Wait for 10s", 0, 1);
  mpu6050.calcGyroOffsets(true);
  lcd.clear();
  printf("Calibration done", 0, 0);
  printf("Waiting 4 Serial", 0, 1);
}

void loop() {
  digitalWrite(Ready, LOW); // Means that Arduino is ready for next command
  if(Serial.available()) {
    digitalWrite(Ready, HIGH); // When this pin is high, it means that the arduino is currently executing a command
    parsedAngle = Serial.parseFloat();  // Angle which the Pi has sent
    lcd.clear();
    if (parsedAngle < 0) { // Debug value is 100.0
      mpu6050.update();  // update rotation values
      printf("Turn right:", 0, 0);
      lcd.print(-parsedAngle);
      targetAngle = parsedAngle + mpu6050.getAngleZ(); // Get Target angle by finding sum of parsedAngle and current angle
      while (targetAngle < mpu6050.getAngleZ()) {
        Drive.writeMicroseconds(2000);
        mpu6050.update();
        printf("Remaining:", 0, 1);
        lcd.print(-(targetAngle - mpu6050.getAngleZ()));
      }
      shoot();
    }
    if (70 > parsedAngle > 0) { 
      mpu6050.update();
      printf("Turn left:", 0, 0);
      lcd.print(parsedAngle);
      targetAngle = parsedAngle + mpu6050.getAngleZ();
      Serial.println(targetAngle);
      while (targetAngle > mpu6050.getAngleZ()) {
        Drive.writeMicroseconds(1000);
        mpu6050.update();
        printf("Remaining:", 0, 1);
        lcd.print(targetAngle - mpu6050.getAngleZ());
      }
      shoot();
    }
    if (parsedAngle >= 70 || parsedAngle == 0) { // Raspberry Pi will send a value of 100.0 to indicate no targets identified
      lcd.clear();
      Drive.writeMicroseconds(1500);
      printf("Targets N/A", 0, 0);
    }
  }

}


// Simpler to use lcd.print functions
void printf(String text, int x, int y) { // Formatted LCD printing
  lcd.setCursor(x, y);
  lcd.print(text);
}

void printf(float text, int x, int y) { // Formatted LCD printing
  lcd.setCursor(x, y);
  lcd.print(text);
}

// Shoot function

void shoot() {
  Drive.writeMicroseconds(1500); // Stop drive motors
  Feed.writeMicroseconds(2000); // Feed
  lcd.clear();
  printf("Shooting!", 0, 0);
  delay(500); // Wait for feed
  Feed.writeMicroseconds(1500); // Stop
}
