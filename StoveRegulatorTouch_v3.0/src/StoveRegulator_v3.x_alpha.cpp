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

 The code drives an Arduino microcontroller based system that automatically 
 controls the air damper on a wood stove in order to maintain a constant
 exhaust pipe temperature.  
 
 System inputs and outputs: 

  1) Exhaust pipe temperature as measured by a thermocouple mounted directly to
     the exhaust pipe. Sensor is K-type thermocouple with a ring attach end
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

// Software Serial pin mapping variables:
const int rxPort = 2;               // RX pin on Arduino
const int txPort = 3;               // TX pin on Arduino

// Thermocouple pin mapping variables and compensation error variable:
const int thermoCsPort = 6;         // CS pin on MAX6675; chip select
const int thermoSoPort = 7;         // SO pin of MAX6675; serial output
const int thermoSckkPort = 8;       // SCK pin of MAX6675; serial clock input
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
SoftwareSerial Serial2(rxPort, txPort); // RX, TX

// Servo calibration settings:
// Servo rate calibration: neutral calibration is 1.0 - adjust value so servo
// arm drives closed damper when damper variable equals 0 (0%)
float servoCalibration = 1.0;
// offset angle for servo angle adjustment
float servoOffset = -20;
// adjust value to define total angular travel of servo so that cable drives
// damper from fully opened to fully closed
float servoRange = 100;

// Temperature variables:       
// Initialize temperature variable (50F = 10C)
int tempF = 50;
// Under this temperature (100F = 38C), the regulation closes the damper
// if end of fire conditions are met
int tempFMin = 100;
// Target temperature (275F = 135C)
int targetTempF = 275;

// PID regulation variables:
float errP = 0.0;          // initialize the proportional term
float errD = 0.0;          // initialize the derivative term
float errI = 0.0;          // initialize the integral term
float errOld = 0.0;        // initialize term used to capture prior cycle ErrP
float kP = 5.0;            // P coefficient of the PID regulation
float tauI = 1000;           // Integral time constant (sec/repeat)
float tauD = 5;              // Derivative time constant (sec/reapeat)
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
int targetDamper = 0;
int diff = 0;
float maxDamper = 100;  // Sets maximum damper setting
float minDamper = 0.0;    // Sets minimum damper setting
float zeroDamper = 0.0;   // Sets zero damper setting
// Note that stove allows some amount of airflow at zero damper

//bool endBuzzer = true;
//bool refillBuzzer = true;

 // Setup temperature history array
float tempHist[6] = {0, 0, 0, 0, 0, 0};

// NOTE : String used to add end chars to the data sent to the Nextion Display
//        and to verify the end of the string for incoming data.

String endChar = String(char(0xff)) + String(char(0xff)) + String(char(0xff));
String dfd  = "";      // data from display
String command = "";   // command from display
String cmd_value = ""; // value from display
bool autoMode = true;  // auto mode flag

// Setup async delay periods 
const unsigned long readPeriod = 1000;
const unsigned long regulationPeriod = 500;
const unsigned long servoPeriod = 300;
const unsigned long serialPrintPeriod = 1000;
unsigned long lastReadTime = 0;
unsigned long lastRegulationTime = 0;
unsigned long lastServoTime = 0;
unsigned long lastPrintTime = 0;


// ****************************************************************************
// Function definitions:
// ****************************************************************************

// Function checks if temp is moving towards set point temp (wood added)
// Returns 'true' for temperature climbing and 'false' for temperature falling
bool woodFilled(int currentTemp) {
  for (int i = 5; i > 0; i--) {
    tempHist[i] = tempHist[i-1];
  }
  tempHist[0] = currentTemp;

  if (float((tempHist[0]+tempHist[1]+tempHist[2])/3.0) >
      float((tempHist[3]+tempHist[4]+tempHist[5])/3.0)) {
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
  // hardware port used for output to serial monitor
  Serial.begin(9600);
  // port for display display communication - SofwareSerial sets pinModes
  Serial2.begin(9600);

  //Serial2.print("baud=28800"); // set baud rate for display
  //Serial2.begin(28800); 

  // use pin 11 vs pin 10 for servo to avoid conflict if using AltSoftSerial
  myservo.attach(11);
  // TODO: calculate center angle for servo and write to servo
  myservo.write(0);    // write 0 deg angle to servo object
  delay(300);         // delay to allow servo to move
  myservo.detach();    // detach servo object
  //pinMode(buzzerPort, OUTPUT); // configure buzzer pin as an output
}

// ****************************************************************************
// Arduino loop function
// ****************************************************************************

void loop() {

// Read the temperature from the thermocouple in degrees F 

// NOTE : ASYNC DELAY should be 220ms min for max6675 readCelsius() function
  if((millis() - lastReadTime) > readPeriod){
  // pattern guards aginst error for millis() overflow condition
    lastReadTime = millis();
    tempF = (int)thermocouple.readFahrenheit();
    if (isnan(tempF)){
      Serial.println("Thermocouple Error."); // report a thermocouple error
    }
    // Write temperature to Nextion display auto and manual screen temp fields 
    Serial2.print("auto_page.n2.val=" + String(tempF) + endChar);
    Serial2.flush();
    Serial2.print("man_page.n2.val=" + String(tempF) + endChar);
    Serial2.flush();
  }
  
// ****************************************************************************  
// Check for and capture command and value sent from the touch screen
// ****************************************************************************

  if (Serial2.available() > 0) {
    dfd += char(Serial2.read());
    Serial.println(dfd); 
    Serial.flush();
    // NOTE : COMMAND is 4 characters after C:C
    // NOTE : RESET dfd if THREE characters received and not C:C
    if (dfd.length()>2 && dfd.substring(0,3)!="C:C") {
      dfd="";
    }
    else {
      // NOTE : If string ends in C?C then command completed
      if (dfd.substring((dfd.length()-3),dfd.length()) == "C?C") {
        // NOTE : Get the command
        command = dfd.substring(3,7);
        // NOTE : Get the value(int or string)
        cmd_value = dfd.substring(7,dfd.length()-3);
        dfd="";
        Serial.println();
        Serial.print(command);
        Serial.print(", ");
        Serial.print(cmd_value);
        Serial.println();
        Serial.println();
        Serial.flush();

        Serial2.flush();
      }
    }
  }

  //if ((millis() - lastRegulationTime) > regulationPeriod) {
  //  lastRegulationTime = millis();

    // Set target temp based on input from Nextion display auto screen
    if (command == "TARG") {
      targetTempF = cmd_value.toInt();
      autoMode = true;
    }
    else if (command == "DAMP") {
      // Manual damper regulation mode
      errI = 0;  // reset integral term based on selection of manual control
      errD = 0;  // reset derivative term based on selection of manual control
      //endBuzzer = true;  // setting to disable end of fire buzzer
      //refillBuzzer = true;  //setting to disable refill buzzer
      // set damper target to value from Nextion display
      targetDamper = cmd_value.toInt();
      autoMode = false;
    }
  
    if (autoMode == true) {
      if (errI < endTrigger) {
        // Automatic PID regulation
        errP = targetTempF - tempF;        // set proportional term
        errI = errI + errP;                // update integral term
        errD = errP - errOld;              // update derivative term
        errOld = errP;
        // Call function that checks if wood is refilled to update tracking array
        woodFilled(tempF);
        // set damper position and limit damper values to physical constraints
        targetDamper = (int)(kP * errP + kI * errI + kD * errD);
        if (targetDamper < minDamper) targetDamper = minDamper;
        if (targetDamper > maxDamper) targetDamper = maxDamper;
        //Refill Alarm
        //if (errI > refillTrigger) {
        // set damper output message, fill
        //  messageDamp = "Dmp " + String(damper) + "%   Fill ";
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
  
        // set damper output message to "end"
        if (tempF < tempFMin) {
          targetDamper = targetDamper; // TODO include zeroDamper;
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
    }
  //}

  // Drive servo and send damper position to Nextion display
  if ((millis() - lastServoTime) > servoPeriod) {
    lastServoTime = millis();
    diff = targetDamper - damper;
    if (diff != 0) {
      myservo.attach(11);
      damper += diff / abs(diff);
      angle = (int)((damper * servoRange) /
              (maxDamper * servoCalibration) + servoOffset);
      myservo.write(angle);
      delay(200);
      myservo.detach();
      // Send damper angle to Nextion display via soft serial tx
      Serial2.print("auto_page.n1.val=" + String(damper) + endChar);
      Serial2.flush();
    }
    else {
      myservo.detach();
    }
  }

  // Regulator model data via serial output
  // Output: tempF, damper%, damper(calculated), damperP, damperI, damperD, errP, errI, errD
  /*
  if ((millis() - lastPrintTime) > serialPrintPeriod) {
    lastPrintTime = millis();

    Serial.print(tempF);
    Serial.print(",");
    Serial.print(damper);
    Serial.print(",");
    Serial.print(targetDamper);
    Serial.print(",");
    Serial.print(angle);
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
  */
}
