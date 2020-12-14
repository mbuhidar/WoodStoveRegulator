/*
 Wood Stove Regulator - Version 2.0
 by Michael Buhidar (GitHub: mbuhidar)

 with thanks to philco for many of the foundational ideas for this project
 (https://www.instructables.com/id/Wood-Stove-Regulator/)

 Sketch drives an Arduino microcontroller based system that auomatically 
 controls the air damper on a wood stove in order to maintain a 
 constant exhaust pipe temperature.  
 
 System inputs: 
  1) Exhaust pipe temperature as measured by a thermocouple 
  mounted directly to the exhaust pipe (sensor is K-type thermocouple with 
  d-ring end connected to Arduino via MAX6675 module.
  2) A variable knob that controls LCD brightness (10K potentiometer).
  3) A variable knob that manually controls the stove damper 
  position between 0% and 100% and places the control system in 
  automatic mode at potentiometer setting above 100% (10K potentiometer). 
  4) FUTURE FEATURE: A variable knob that controls the temperature target for
  the stove exhaust pipe.  Range from 200-400F.

 System outputs:
  1) LCD display showing:
     a) Exhaust pipe temperature on 1st line of LCD.
     b) Stove control knob potentiometer setting on 1st line of LCD.
     c) Damper position when in manual mode or in automatic
        mode while stove has adequate wood loaded on 2nd line of LCD.
     d) Message to refill the stove on 2nd line if wood is low or
        message that fire is ended if wood is not refilled.
  2) Physical control of the stove air damper transmitted from a 
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
float temperatureMin = 38; // under this temperature (38C = 100F), the regulation closes the damper if end of fire conditions are met
float targetTempC = 135; // the target temperature as measured by the thermocouple (135 C = 275 F)
float errP = 0.0;          // initialize the proportional term
float errD = 0.0;          // initialize the derivative term
float errI = 0.0;          // initialize the integral term
float errOld = 0.0;        // initialize term used to capture prior cycle ErrP
float kP = 5;              // P coefficient of the PID regulation
float kI = 0.0005;         // I coefficient of the PID regulation
float kD = 0.00005;        // D coefficient of the PID regulation

float refillTrigger = 5000;// refillTrigger used to notify need of a wood refill
float endTrigger = 25000;  // closeTrigger used to close damper at end of combustion

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

String messageTemp; // Initialize message for temperature output to lcd
String messageDamp; // Initialize message for damper setting output to lcd

boolean endBuzzer = true;
boolean refillBuzzer = true;



// Utility function to convert temperature in C to F
int tempF(int tempC){
 return tempC * 9 / 5 + 32;
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
  
  delay(500); 
  
  if (temperature == -1) {
    Serial.println("Thermocouple Error."); // if temperature is read as -1 report a thermocouple error to serial output
  } else {

    pot = analogRead(potPort); // reads the value of the potentiometer (value between 0 and 1023)
    pot = map(pot, 0, potMax, 0, 105); // scale potentiometer reading to 0 to 105%; note that setting above 100% invokes automatic mode
    
    if (pot <= potRelMax ) {  
      // Manual damper regulation mode if potentiometer reads 100% or less
      errI = 0;  // reset integral term based on user intent for manual control
      errD = 0;  // reset derivative term based on user intent for manual control
      endBuzzer = true;  // setting to disable end of fire buzzer
      refillBuzzer = true;  //setting to disable refill buzzer
      damper = round(pot * maxDamper / 100);  // scales damper setting according to max setting
      messageDamp = "Damper=" + String(damper) + "% Man"; // set damper output message
    }
    else
    { 
      if (errI < endTrigger) {
        // Automatic PID regulation
        errP = targetTempC - temperature;  // set proportional term
        errI = errI + errP;                // update integral term
        errD = errP - errOld;              // update derivative term
        errOld = errP;
        // set damper position and limit damper values to physical constraints
        damper = kP * errP + kI * errI + kD * errD;
        if (damper < minDamper) damper = minDamper;
        if (damper > maxDamper) damper = maxDamper;

        messageDamp = "Damper=" + String(damper) + "% Auto";

        //Refill Alarm
        if (errI > refillTrigger) {
          messageDamp = "Damper=" + String(damper) + "% Fill";
          if (refillBuzzer) {
            for (int i = 0; i < buzzerRefillRepeat; i++) {
              tone(buzzerPort, buzzerRefillFrequency);
              delay(buzzerRefillDelay);
              noTone(buzzerPort);
              delay(buzzerRefillDelay);
            }
            refillBuzzer = false;
          }
          if (temperature > targetTempC) {
            errI = 0;  // reset integral term after wood refill
            errD = 0;  // reset derivative term after wood refill
            refillBuzzer = true;  // reset buzzer toggle
          }
        }

      }

      else {
        // End of combustion condition for errI >= endTrigger
        messageDamp = "Damper=" + String(damper) + "% End";
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
      }
    }

    // Display messages on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    messageTemp = "Tmp=" + String(tempF(temperature)) + "F" + " Pot=" + round(pot) + "%";
    lcd.print(messageTemp);  // output temperature message to upper left quadrant of lcd
    lcd.setCursor(0, 1);    
    lcd.print(messageDamp);  // output damper message to upper right quadrant of lcd


    // Drive servo and print damper position to the lcd
    diff = damper - oldDamper;
    if (abs(diff) > 2) {  // move servo only if damper position change is more than 2%
      lcd.print(" x");    // print x after damper message to indicate damper is actively changing
      delay(500);
      myservo.attach(10);
      if (diff > 0) {  // action if damper should be opened
        for (int i = 0; i < diff; i++) {
          angle = (oldDamper + i + 1) * servoAngle / (maxDamper * servoCalibration) - servoOffset;
          myservo.write(angle);
          delay(200);
        }
      }
      
      if (diff < 0) {  // action if damper should be closed
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
    Serial.print(round(damper/maxDamper * 100));
    Serial.print(",");
    Serial.print(tempF(temperature));
    Serial.print(",");
    Serial.print(errP);
    Serial.print(",");
    Serial.print(errI);
    Serial.print(",");
    Serial.print(errD);
    Serial.print(",");    
    Serial.println();

    delay(4000);

  }
}
