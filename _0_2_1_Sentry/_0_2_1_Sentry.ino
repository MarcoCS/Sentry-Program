// Created By Marco Sin
// Purpose of program: Control Robot
// Ver. 0.2.1 (As of April 20)
/*
Program receives incoming float values over serial.  Then handles
motor controls and uses a gyroscope to precisely aim and shoot at
stationary targets
*/


/*
Added: multi-data type exception to function "printf" Ver. 0.2.0

Bugfixes: Added a garbage value to be displayed when no targets are present. ver. 0.2.1
*/
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <LCD.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
float f;
int timer = 10;

void setup() {
  Serial.begin(115200);
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  while (timer-- != 0) { // 10s warmup to allow for adequate calibration of Gyroscope
    lcd.clear();
    printf("Countdown", 0, 0);
    printf(String(timer), 0, 1);
    delay(1000); // Wait 10s
  }
  
}

void loop() {
  if(Serial.available()) {
    f = Serial.parseFloat();
    lcd.clear();
    if (70 > f > 0) { // Debug value is 100.0
      printf("Turn right:", 0, 0);
      printf(f, 11, 0);
    }
    else if (f < 0) {  // For some reason Python program sends this as a default value when there is no target
      printf("Turn left:", 0, 0);
      printf(f, 11, 0);
    }
    else if (f >= 70) {
      printf("Targets N/A", 0, 0);
    }
  }

}

void printf(String text, int x, int y) { // Formatted LCD printing
  lcd.setCursor(x, y);
  lcd.print(text);
}

void printf(float text, int x, int y) { // Formatted LCD printing
  lcd.setCursor(x, y);
  lcd.print(text);
}
