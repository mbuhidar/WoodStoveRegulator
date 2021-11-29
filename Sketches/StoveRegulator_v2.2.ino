/*
 Wood Stove Regulator - Version 2.2
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

 Sketch drives an Arduino microcontroller-based device that automatically 
 controls the air damper position on a wood stove in order to maintain a 
 constant exhaust pipe temperature.  
 
 System inputs: 
  1) Exhaust pipe temperature as measured by a thermocouple 
  mounted directly to the exhaust pipe (K-type thermocouple with 
  d-ring end connected to Arduino via MAX6675 module.
  2) A variable knob that controls LCD brightness (10K potentiometer).
  3) A variable knob that manually controls the stove damper 
  position between 0% and 100% for the first half of the knob rotation range
  and places the control system in automatic mode for potentiometer settings
  greater than half the rotation range. In automatic mode, the knob controls
  setting the stove target temperature between 100 C (212 F) and 200 C (392 F). 

 System outputs:
  1) LCD display showing:
     a) Exhaust pipe temperature and target temperature on 1st line of LCD.
     b) Alternating message on 2nd line of LCD showing A. damper / 
     potentiometer position and B. damper / mode - auto or manual.  
  2) Physical control of the stove air damper is transmitted from a 
     servo motor mounted inside the control unit with force and 
     position being transferred via sheathed control cable connected 
     between the servo and the stove damper lever.
  3) Passive buzzer that provides an audible alert when wood needs to be added
     or when stove damper will be closed due to end of fire event.
*/

// Include libraries for max6675, servo, LCD, and math.
#include <max6675.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <math.h>

// Thermocouple pin mapping variables and compensation error variable:
int thermoCsPort = 6;               // CS pin on MAX6675; chip select
int thermoSoPort = 7;               // SO pin of MAX6675; serial output
int thermoSckkPort = 8;             // SCK pin of MAX6675; serial clock input
float error = 0.0;                  // Temperature compensation error

// Buzzer setup variables:
int buzzerPort = 9;               // Buzzer port id
int buzzerRefillFrequency = 1900; // Buzzer tone frequency for refill alarm
int buzzerRefillRepeat = 3;       // Number of refill alarm tones
int buzzerRefillDelay = 1000;     // Delay between refill alarm tones
int buzzerEndFrequency = 950;     // Buzzer tone frequency for end of fire damper close alarm
int buzzerEndRepeat = 2;          // Number of tones for end of fire damper close alarm
int buzzerEndDelay = 2000;        // Delay of tone for end of fire damper close alarm

// Potentiometer variables
int servoPort = 0;
int potPort = 0;

// LCD port variables
int lcdPort1 = 12;
int lcdPort2 = 11;
int lcdPort3 = 5;
int lcdPort4 = 4;
int lcdPort5 = 3;
int lcdPort6 = 2;

// Device objects - create servo, therocouple, and lcd objects 
Servo myservo;
MAX6675 thermocouple(thermoSckkPort, thermoCsPort, thermoSoPort); 
LiquidCrystal lcd(lcdPort1, lcdPort2, lcdPort3, lcdPort4, lcdPort5, lcdPort6); 

// Servo calibration settings
float servoCalibration = 1.0;  // 1.0 is neutral cal - adjust value so servo arm drives closed damper when damper variable equals 0 (0%)
float servoOffset = 20;  // offset angle for servo angle adjustment
float servoAngle = 100;  // adjust value to define total angular travel of servo so that cable drives damper from fully opened to fully closed

/*
  Note - regulation variables (Based on PID):
      damper = kP * errP + kI * errI + kD * errD
      errP = targetTempC - temperature
      errI = errP integral
      errD = errP derivative
*/

int temperature = 0;       // initialize temperature variable for C
int temperatureMin = 38; // under this temperature (38C = 100F), the regulation closes the damper if end of fire conditions are met
int targetTempC = 135;   // the target temperature as measured by the thermocouple (135 C = 275 F)
float errP = 0.0;          // initialize the proportional term
float errD = 0.0;          // initialize the derivative term
float errI = 0.0;          // initialize the integral term
float errOld = 0.0;        // initialize term used to capture prior cycle ErrP
float kP = 5.0;            // Overall gain coefficient and P coefficient of the PID regulation
float tauI = 1000;         // Integral time constant (sec/repeat)
float tauD = 5;            // Derivative time constant (sec/reapeat)
float kI = kP/tauI;        // I coefficient of the PID regulation
float kD = kP/tauD;        // D coefficient of the PID regulation

float refillTrigger = 5000;// refillTrigger used to notify need of a wood refill
float endTrigger = 25000;  // closeTrigger used to close damper at end of combustion

int pot_raw = 0;
int pot = 0;
int oldPot = 0;
float potMax = 1023.0;   // Potentiometer calibration
int potRelMax = 100;     // Potentiometer value above which the regulator runs in automatic mode

int angle = 0;
int damper = 0;
int oldDamper = 0;
int diff = 0;
float maxDamper = 100.0;  // Sets maximum damper setting
float minDamper = 0.0;    // Sets minimum damper setting
float zeroDamper = 0.0;   // Sets zero damper setting - note that stove allows some amount of airflow at zero damper

String messageTemp;    // Initialize message for temperature line output to lcd
String messageDamp;    // Initialize message for damper line output to lcd

bool endBuzzer = true;
bool refillBuzzer = true;
bool oddLoop = true;

int TempHist[6] = {0, 0, 0, 0, 0, 0}; // Set temperature history array


// Utility function to convert temperature in C to F
int tempF(int tempC){
 return tempC * 9 / 5 + 32;
}


// Utility function to check if wood has been filled and stove is building temp towards set point temp
// Returns 'true' for refilled and temperature climbing or 'false' for temperature falling
bool WoodFilled(int CurrentTemp) {
  for (int i = 5; i > 0; i--) {
    TempHist[i] = TempHist[i-1];
  }
  TempHist[0] = CurrentTemp;

  if (float((TempHist[0]+TempHist[1]+TempHist[2])/3.0) > float((TempHist[3]+TempHist[4]+TempHist[5])/3.0)) {
    return true;
  }
  else {
    return false;
  }
}


// arduino setup function
void setup() {
  Serial.begin(9600);  // opens serial port, sets data rate to 9600 bps
  myservo.attach(10);  // attach servo object to pin 10
  myservo.write(0);    // write 0 deg angle to servo object
  myservo.detach();    // detach servo object from its pin
  lcd.begin(16, 2);    // initializes 1602 lcd screen for 16 columns and 2 rows
  pinMode(buzzerPort, OUTPUT); // configure buzzer pin as an output
}



// arduino loop function
void loop() {

  temperature = thermocouple.readCelsius();  // read thermocouple temp in C
  
  delay(1000); // allow one second for sensor read and settle
  
  if (temperature == -1) {
    Serial.println("Thermocouple Error."); // if temperature is read as -1 report a thermocouple error to serial output
  } else {

    pot_raw = analogRead(potPort); // reads the value of the potentiometer (value between 0 and 1023)

    /* scale potentiometer reading to 0 to 200%; note that setting above 100% invokes automatic mode.
       Until another pysical input can be added to the unit, the pot range from 100-200% will be used 
       for setting target temp
    */
    pot = map(pot_raw, 0, potMax, 0, 200);
    targetTempC = map(pot, 100, 200, 100, 200);  // Maps pot % range to target temp range of 212 to 392 F
    targetTempC = constrain(targetTempC, 100, 200);  // Limit target temp to specified range
    
    if (pot <= potRelMax ) {  
      // Manual damper regulation mode if potentiometer reads 100% or less
      errI = 0;  // reset integral term based on user intent for manual control
      errD = 0;  // reset derivative term based on user intent for manual control
      endBuzzer = true;  // setting to disable end of fire buzzer
      refillBuzzer = true;  //setting to disable refill buzzer
      damper = round(pot * maxDamper / 100);  // scales damper setting according to max setting
      messageDamp = "Dmp " + String(damper) + "%    Man "; // set damper output message, manual
    }
    else { 
      if (errI < endTrigger) {
        // Automatic PID regulation
        errP = targetTempC - temperature;  // set proportional term
        errI = errI + errP;                // update integral term
        errD = errP - errOld;              // update derivative term
        errOld = errP;
        WoodFilled(temperature);  // Call function that checks if wood is refilled to update array

        // set damper position and limit damper values to physical constraints
        damper = kP * errP + kI * errI + kD * errD;
        if (damper < minDamper) damper = minDamper;
        if (damper > maxDamper) damper = maxDamper;

        messageDamp = "Dmp " + String(damper) + "%   Auto "; // set damper output message, auto

        //Refill Alarm
        if (errI > refillTrigger) {
          messageDamp = "Dmp " + String(damper) + "%   Fill "; // set damper output message, fill
          if (refillBuzzer) {
            for (int i = 0; i < buzzerRefillRepeat; i++) {
              tone(buzzerPort, buzzerRefillFrequency);
              delay(buzzerRefillDelay);
              noTone(buzzerPort);
              delay(buzzerRefillDelay);
            }
            refillBuzzer = false;
          }
          if (WoodFilled(temperature)) {
            errI = 0;  // reset integral term after wood refill
            refillBuzzer = true;  // reset buzzer toggle
          }
        }

      }

      else {
        // End of combustion condition for errI >= endTrigger

        messageDamp = "Dmp " + String(damper) + "%    End "; // set damper output message, end

        if (temperature < temperatureMin) {
          damper = zeroDamper;
        }
        if (endBuzzer) {
          for (int i = 0; i < buzzerEndRepeat; i++) {
            tone(buzzerPort, buzzerEndFrequency);
            delay(buzzerEndDelay);
            noTone(buzzerPort);
            delay(buzzerEndDelay);
          }
          endBuzzer = false;
        }
        // Check if wood has been added and reset errI if so
        if (WoodFilled(temperature)) {
          errI = 0;  // reset integral term after wood refill
        }
      }
    }

    // Display messages on 16X2 LCD
    /*
    0123456789ABCDEF
    Tmp=XXXF Pot=XXX%  - line 1, original message
    Damper=XX% Auto x  - line 2, original message

    0123456789ABCDEF
    Temp XXXF / XXXF  - line 1, actual temp / target temp
    Dmp XXX% Pot XXX  - line 2, rotating message A 
    Dmp XXX%  Auto x  - line 2, rotating message B 

    */
    lcd.clear();
    lcd.setCursor(0, 0);
    messageTemp = "Temp " + String(tempF(temperature)) + "F / " + String(tempF(targetTempC)) + "F";
    if (oddLoop) {
      messageDamp = "Dmp " + String(damper) + "% Pot " + round(pot); // set alt damper output message
    }
    lcd.print(messageTemp);  // output temperature message to upper left quadrant of lcd
    lcd.setCursor(0, 1);
    lcd.print(messageDamp);  // output damper message to upper right quadrant of lcd


    // Drive servo and print damper position to the lcd
    diff = damper - oldDamper;
    if (abs(diff) > 2) {  // move servo only if damper position change is more than 2%
      if (!oddLoop) {
        lcd.print("x");   // print x after damper message to indicate active movement
      }
      delay(500);
      myservo.attach(10);
      if (diff > 0) {  // action if damper should be moved in the opened direction
        for (int i = 0; i < diff; i++) {
          angle = (oldDamper + i + 1) * servoAngle / (maxDamper * servoCalibration) - servoOffset;
          myservo.write(angle);
          delay(200);
        }
      }
      
      if (diff < 0) {  // action if damper should be moved in the closed direction
        for (int i = 0; i < abs(diff); i++) {
          angle = (oldDamper - i - 1) * servoAngle / (maxDamper * servoCalibration) - servoOffset;
          myservo.write(angle);
          delay(200);
        }
      }
          
      myservo.detach();

      oldPot = pot;
      oldDamper =  damper;

    }

    // Regulator model data via serial output
    // Output: tempC, tempF, damper%, damper(calculated), damperP, damperI, damperD, errP, errI, errD

    Serial.print(temperature);
    Serial.print(",");
    Serial.print(tempF(temperature));
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

    // Toggle oddLoop that controls display message on line 2
    oddLoop = !oddLoop;
    // Delay between loop cycles
    delay(4000);

  }
}
