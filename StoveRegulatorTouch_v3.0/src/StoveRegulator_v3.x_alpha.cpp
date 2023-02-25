/*
 Wood Stove Regulator - Version 3.x
 Copyright (C) 2020  Michael Buhidar (GitHub: mbuhidar)
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 For a copy of the GNU General Public License, 
 see <https://www.gnu.org/licenses/>.

 With thanks to philco for many of the foundational ideas for this project
 (https://www.instructables.com/id/Wood-Stove-Regulator/)

 The code drives an Arduino microcontroller based system that auomatically 
 controls the air damper on a wood stove in order to maintain a constant
 exhaust pipe temperature.  
 
 System inputs and outputs: 
  1) Exhaust pipe temperature as measured by a thermocouple mounted directly to
     the exhaust pipe (sensor is K-type thermocouple with a d-ring end
     connected to an Arduino (Uno in this case) via MAX6675 module.

  2) A touch screen display (Nextion NX3224T024_011) that has two modes - 
     automatic and manual.  
     - Automatic Mode: 
          Screen displays the damper position as a percentage (0-100%), the
          exhaust pipe temperature as an integer (deg F), and the target
          temperature as an integer (deg F). The screen also has in input
          slider bar that can be used to set the target temperature. The damper
          position is automatically controlled by the Arduino microcontroller
          based on the exhaust pipe temperature in this mode. There is a button
          that can be pressed to move to manual mode and display the manual
          mode screen.  
     - Manual Mode:
          Screen displays the damper position as a percentage (0-100%) and the
          exhaust pipe temperature as an integer (deg F). The target
          temperature is greyed out in this mode since it cannot be set on this
          screen. The damper position in this mode is is directly controlled by
          input slider bar on the screen. There is a button that can be pressed
          to move to automatic mode and display the automatic mode screen.
     - TODO: Both modes:
          There is an indicator that flashed when wood needs to be added to the 
          stove. A buzzer is also used to provide an audible alert when wood
          needs to be added.

  3) Physical control of the stove air damper transmitted from a servo motor 
     mounted inside the control unit with force and position being transferred
     via sheathed control cable connected between the servo and the stove
     damper lever.

  4) TODO: Passive buzzer that provides an audible alert when wood needs to be
     added or when stove damper will be closed due to end of fire event.
*/

// Include libraries for Arduino, max6675, servo, math, and software serial.

#include <Arduino.h>
#include <max6675.h>
#include <Servo.h>
#include <math.h>
#include <SoftwareSerial.h>

// ****************************************************************************
// Device setup and initialization:
// ****************************************************************************

// Software Serial port setup:
SoftwareSerial Serial2(2, 3); // RX, TX

// Thermocouple pin mapping variables and compensation error variable:
int thermoCsPort = 6;               // CS pin on MAX6675; chip select
int thermoSoPort = 7;               // SO pin of MAX6675; serial output
int thermoSckkPort = 8;             // SCK pin of MAX6675; serial clock input
float error = 0.0;                  // Temperature compensation error

// Buzzer setup variables:
/*
// Buzzer port id
int buzzerPort = 9;
// Buzzer tone frequency for refill alarm
int buzzerRefillFrequency = 1900;
// Number of refill alarm tones
int buzzerRefillRepeat = 3;
// Delay between refill alarm tones
int buzzerRefillDelay = 1000;
// Buzzer tone frequency for end of fire damper close alarm
int buzzerEndFrequency = 950;
// Number of tones for end of fire damper close alarm
int buzzerEndRepeat = 2;
// Delay of tone for end of fire damper close alarm
int buzzerEndDelay = 2000;
*/

// Device objects - create servo and thermocouple objects 
Servo myservo;
MAX6675 thermocouple(thermoSckkPort, thermoCsPort, thermoSoPort); 

// Servo calibration settings:

// Servo rate calibration: neutral calibration is 1.0 - adjust value so servo
// arm drives closed damper when damper variable equals 0 (0%)
float servoCalibration = 1.0;
// offset angle for servo angle adjustment
float servoOffset = 20;
// adjust value to define total angular travel of servo so that cable drives
// damper from fully opened to fully closed
float servoAngle = 100;

// Temperature variables: 
int tempF = 32;       // initialize temperature variable (32F = 0C)
int tempFMin = 100; // under this temperature (100F = 38C), the regulation closes the damper if end of fire conditions are met
int targetTempF = 275;   // the target temperature as measured by the thermocouple (275F = 135C)

// PID regulation variables:
float errP = 0.0;          // initialize the proportional term
float errD = 0.0;          // initialize the derivative term
float errI = 0.0;          // initialize the integral term
float errOld = 0.0;        // initialize term used to capture prior cycle ErrP
float kP = 5.0;            // Overall gain coefficient and P coefficient of the PID regulation
float tauI = 1000;         // Integral time constant (sec/repeat)
float tauD = 5;            // Derivative time constant (sec/reapeat)
float kI = kP/tauI;        // I coefficient of the PID regulation
float kD = kP/tauD;        // D coefficient of the PID regulation

// Refill and end of fire variables:
// refillTrigger used to notify need of a wood refill
//float refillTrigger = 5000;
// closeTrigger used to close damper at end of combustion
float endTrigger = 25000;

// Damper variables:
int angle = 0;
int damper = 0;
int oldDamper = 0;
int diff = 0;
float maxDamper = 100.0;  // Sets maximum damper setting
float minDamper = 0.0;    // Sets minimum damper setting
float zeroDamper = 0.0;   // Sets zero damper setting - note that stove allows some amount of airflow at zero damper

//String messageTemp;    // Initialize message for temperature line output to lcd
//String messageDamp;    // Initialize message for damper line output to lcd

//bool endBuzzer = true;
//bool refillBuzzer = true;
//bool oddLoop = true;

 // Set temperature history array
int TempHist[6] = {0, 0, 0, 0, 0, 0};

// NOTE : Below is a String used to add to the data sent to the Nextion Display
//        and to verify the end of the string for incoming data.

String endChar = String(char(0xff)) + String(char(0xff)) + String(char(0xff));
String dfd  = ""; // data from display
String command = ""; // command from display
String cmd_value = ""; // value from display

// NOTE : Initial Async Delay 
unsigned long last_millis = 0; // NOTE : 4,294,967,295
int	period = 3000;

// ****************************************************************************
// Function definitions:
// ****************************************************************************

// Utility function to check if wood has been filled and stove is building temp towards set point temp
// Returns 'true' for refilled and temperature climbing or 'false' for temperature falling
bool woodFilled(int currentTemp) {
  for (int i = 5; i > 0; i--) {
    tempHist[i] = tempHist[i-1];
  }
  tempHist[0] = currentTemp;

  if (float((tempHist[0]+tempHist[1]+tempHist[2])/3.0) > float((tempHist[3]+tempHist[4]+tempHist[5])/3.0)) {
    return true;
  }
  else {
    return false;
  }
}


// ****************************************************************************
// Arduino setup
// ****************************************************************************

void setup() {
  Serial.begin(9600);  // serial port for debugging Arduino and temp alorithm
  Serial2.begin(9600); // serial port for communicating with touch screen
  myservo.attach(10);  // attach servo object to pin 10
  myservo.write(0);    // write 0 deg angle to servo object
  myservo.detach();    // detach servo object from its pin
  //pinMode(buzzerPort, OUTPUT); // configure buzzer pin as an output
}


// ****************************************************************************
// Arduino loop function
// ****************************************************************************

void loop() {

// Read the temperature from the thermocouple in degrees F 

// NOTE : ASYNC DELAY - 220ms for max6675 readCelsius() function
  if((millis() - last_millis) > 220){ // guards aginst millis() overflow
    tempF = thermocouple.readFahrenheit();
    if (isnan(tempF)){
      Serial.println("Thermocouple Error."); // report a thermocouple error
    }
  }
  last_millis = millis();
  
// ****************************************************************************  
// Check for and capture command and value sent from the touch screen
// ****************************************************************************

  dfd += char(Serial2.read());
  // NOTE : COMMAND is 4 characters after C:C
  // NOTE : RESET dfd if THREE characters received and not C:C
  if(dfd.length()>3 && dfd.substring(0,3)!="C:C") dfd="";
  else{
    // NOTE : If string ends in C?C then command completed
    if(dfd.substring((dfd.length()-3),dfd.length()) == "C?C"){
      // NOTE : Get the command
      command = dfd.substring(3,7);
      // NOTE : Get the value(int or string)
      cmd_value = dfd.substring(7,dfd.length()-3);
      // Set target temp based on display input
      if(command == "TARG"){
        targetTempF = cmd_value.toInt();
        // Write temperature to Nextion display auto page
        Serial2.print("auto_page.n2.val=" + String(tempF) + endChar);
      }
      dfd="";
    }
  }

  if (command == "DAMP") {  
    // Manual damper regulation mode
    errI = 0;  // reset integral term based on user intent for manual control
    errD = 0;  // reset derivative term based on user intent for manual control
    //endBuzzer = true;  // setting to disable end of fire buzzer
    //refillBuzzer = true;  //setting to disable refill buzzer
    damper = cmd_value.toInt();  // scales damper setting according to max setting
    // Write temperature to Nextion display manual page
    Serial2.print("man_page.n2.val=" + String(tempF) + endChar);
  }
  else { 
    if (errI < endTrigger) {
      // Automatic PID regulation
      errP = targetTempF - tempF;        // set proportional term
      errI = errI + errP;                // update integral term
      errD = errP - errOld;              // update derivative term
      errOld = errP;
      // Call function that checks if wood is refilled to update array
      woodFilled(tempF);

      // set damper position and limit damper values to physical constraints
      damper = kP * errP + kI * errI + kD * errD;
      if (damper < minDamper) damper = minDamper;
      if (damper > maxDamper) damper = maxDamper;

      // Send damper position to Nextion display
      Serial2.print("auto_page.n1.val=46" + endChar);

      //Refill Alarm
      //if (errI > refillTrigger) {
      //  messageDamp = "Dmp " + String(damper) + "%   Fill "; // set damper output message, fill
      //  if (refillBuzzer) {
      //    for (int i = 0; i < buzzerRefillRepeat; i++) {
      //      tone(buzzerPort, buzzerRefillFrequency);
      //      delay(buzzerRefillDelay);
      //      noTone(buzzerPort);
      //      delay(buzzerRefillDelay);
      //    }
      //    refillBuzzer = false;
      //  }
      //  if (woodFilled(tempF)) {
      //    errI = 0;  // reset integral term after wood refill
      //    refillBuzzer = true;  // reset buzzer toggle
      //  }
      }

    else {
      // End of combustion condition for errI >= endTrigger

      //messageDamp = "Dmp " + String(damper) + "%    End "; // set damper output message, end

      if (tempF < tempFMin) {
        damper = zeroDamper;
      }
      //if (endBuzzer) {
      //  for (int i = 0; i < buzzerEndRepeat; i++) {
      //    tone(buzzerPort, buzzerEndFrequency);
      //    delay(buzzerEndDelay);
      //    noTone(buzzerPort);
      //    delay(buzzerEndDelay);
      //  }
      //  endBuzzer = false;
      //}

      // Check if wood has been added and reset errI if so
      if (woodFilled(tempF)) {
        errI = 0;  // reset integral term after wood refill
      }
    }

  // Drive servo and print damper position to the lcd
  diff = damper - oldDamper;
  if (abs(diff) > 2) {  // move servo only if damper position change is more than 2%
    delay(500);
    myservo.attach(10);
    if (diff > 0) {  // action if damper should be moved in the opened direction
      for (int i = 0; i < diff; i++) {
        angle = (oldDamper + i + 1) * servoAngle / (maxDamper * servoCalibration) - servoOffset;
        myservo.write(angle);
        // Send damper angle to Nextion display
        Serial2.print("auto_page.n1.val=" + String(angle) + endChar);
        delay(200);
      }
    }
    if (diff < 0) {  // action if damper should be moved in the closed direction
      for (int i = 0; i < abs(diff); i++) {
        angle = (oldDamper - i - 1) * servoAngle / (maxDamper * servoCalibration) - servoOffset;
        myservo.write(angle);
        // Send damper angle to Nextion display
        Serial2.print("auto_page.n1.val=" + String(angle) + endChar);
        delay(200);
      }
    }
    myservo.detach();
    oldDamper =  damper;
  }

  // Regulator model data via serial output
  // Output: tempF, damper%, damper(calculated), damperP, damperI, damperD, errP, errI, errD

  Serial.print(tempF);
  Serial.print(",");
  Serial.print(round(damper));
  Serial.print(",");
  Serial.print(round(kP*errP + kI*errI + kD+errD));
  Serial.print(",");
  Serial.print(round(kP*errP));
  Serial.print(",");
  Serial.print(round(kI*errI));
  Serial.print(",");
  Serial.print(round(kD*errD));
  Serial.print(",");  
  Serial.print(errP);
  Serial.print(",");
  Serial.print(errI);
  Serial.print(",");
  Serial.print(errD);
  Serial.print(",");    
  Serial.println();

}