/*
 Wood Stove Regulator
 by Michael Buhidar (GitHub: buhidar)

 System drives an Arduino microcontroller based system that auomatically 
 controls the air damper on a wood stove in order to maintain a 
 constant exhaust pipe temperature.  
 
 System inputs: 
  1) Exhaust pipe temperature as measured by a thermocouple 
  mounted directly to the exhaust pipe (K-type thermocouple with 
  d-ring end connected to Arduino via MAX6675 module.
  2) A variable knob that controls LCD brightness (10K potentiometer).
  3) A variable knob that manually controls the stove damper 
  position between 0% and 99% and places the control system in 
  automatic mode at potentiometer setting of 100% (10K potentiometer). 

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

// Thermocouple pins mapping:
int thermoCsPort = 6;               // CS pin on MAX6675; chip select
int thermoSoPort = 7;               // SO pin of MAX6675; serial output
int thermoSckkPort = 8;             // SCK pin of MAX6675; serial clock input

// int units = 1;                      // Toggle for readout temp units (0 = ˚F, 1 = ˚C) 
float error = 0.0;                  // Temperature compensation error

// Buzzer variables:
int buzzerPort = 9;                 // Buzzer port
int buzzerRefillFrequency = 1900;   // Buzzer tone frequency for refill alarm
int buzzerRefillRepeat = 3;         // Number of refill alarm tones
int buzzerRefillDelay = 1000;       // Delay of refill alarm tone
int buzzerCloseFrequency = 950;    // Buzzer tone frequency for end of fire damper close alarm
int buzzerCloseRepeat = 2;          // Number of tones for end of fire damper close alarm
int buzzerCloseDelay = 2000;        // Delay of tone for end of fire damper close alarm

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

// Device objects
Servo myservo;  // create servo object to control the servo
MAX6675 thermocouple(thermoSckkPort, thermoCsPort, thermoSoPort); 
LiquidCrystal lcd(lcdPort1, lcdPort2, lcdPort3, lcdPort4, lcdPort5, lcdPort6);

// Servo settings
float servoCalibration = 1.0;  // 1.0 is neutral cal - adjust value so servo arm drives closed damper when damper variable equals 0 (0%)
float servoOffset = 20;  // offset angle for servo angle adjustment
float servoAngle = 100;  // adjust value to define total angular travel of servo so that cable drives damper from fully opened to fully closed

// Regulation variables (Based on PID):
// damper = kP * errP + kI * errI + kD * errD
// errP = targetTempC - temperature
// errI = errP integral
// errD = errP derivative
int temperature = 0;  // initialize temperature variable for C
int temperatureF = 32; // initialize temperature variable for F - only used for display output
float temperatureMin = 38; // under this temperature (38C = 100F), the regulation starts an integral measure to estimate end of fire and close the damper
float targetTempC = 121.1; // the target temperature as measured by the thermocouple (121.1 C = 250 F)
float errP = 0.0;
float errD = 0.0;
float errI = 0.0;
float errOld = 0.0;
float kP = 5;              // P parameter of the PID regulation
float kI = 0.0005;         // I parameter of the PID regulation
float kD = 0.00005;        // D parameter of the PID regulation

float refillTrigger = 5000;    // refillTrigger used to notify need of a wood refill
float closeTrigger = 15000;    // closeTrigger used to close damper at end of combustion

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

String messageTemp; // Initialize message for temperature output
String messageDamp; // Initialize message for damper setting output

boolean closeBuzzer = true;
boolean refillBuzzer = true;


void setup() {
  Serial.begin(9600);  // opens serial port, sets data rate to 9600 bps
  myservo.attach(10);  // attach servo variable to pin 10
  myservo.write(0);    // write 0 deg angle to servo
  myservo.detach();    // detach servo variable from its pin
  lcd.begin(16, 2);    // initializes 1602 LCD screen for 16 columns and 2 rows
  pinMode(buzzerPort, OUTPUT); // configures buzzer pin as an output
}

void loop() {

  temperature = thermocouple.readCelsius();
  temperatureF = temperature * 9 / 5 + 32;  // convert C to F
  
  delay(500); 
  
  if (temperature == -1) {
    Serial.println("Thermocouple Error!!"); // Temperature is read as -1 and there is a thermocouple error
  } else {

    pot = analogRead(potPort); // reads the value of the potentiometer (value between 0 and 1023)
    pot = map(pot, 0, potMax, 0, 105); // scale potentiometer reading to 0 to 105%; note that setting above 100% invokes automatic mode
    
    if (pot <= potRelMax ) {
      // Manual damper regulation mode
      damper = round(pot * maxDamper / 100);  // scales damper setting according to max setting
      errI = 0;
      errD = 0;
      closeBuzzer = true;
      refillBuzzer = true;
      messageDamp = "Damper=" + String(damper) + "% Man";
    }
    else
    { 
      if (errI < closeTrigger) {
        // Automatic PID regulation
        errP = targetTempC - temperature;
        errI = errI + errP;
        errD = errP - errOld;
        damper = kP * errP + kI * errI + kD * errD;
        errOld = errP;

        // Limit values to physical constraints ..
        if (damper < minDamper) damper = minDamper;
        if (damper > maxDamper) damper = maxDamper;

        // Close and end fire
        // if (errI >= closeTrigger) errI = closeTrigger;
        // 
        if (temperature < temperatureMin) errI = 0;  // reset errI if temperature is higher than minimum cutoff temperature
        // if (errI >= closeTrigger) damper = zeroDamper;

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
            errI = 0;
            errD = 0;
            refillBuzzer = true;
          }
        }

      }

      else {
        // Close Damper if end of combustion
        messageDamp = "Damper=" + String(damper) + "% End";

        if (closeBuzzer) {
          for (int i = 0; i < buzzerCloseRepeat; i++) {
            tone(buzzerPort, buzzerCloseFrequency);
            delay(buzzerCloseDelay);
            noTone(buzzerPort);
            delay(buzzerCloseDelay);
          }
          closeBuzzer = false;
        }
      }
    }

    // Display message on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    messageTemp = "Tmp=" + String(temperatureF) + "F" + " Pot=" + round(pot) + "%";
    lcd.print(messageTemp);
    lcd.setCursor(0, 1);
    lcd.print(messageDamp);
    // Serial Plotter
    Serial.print(round(damper/maxDamper * 100));
    Serial.print(" ");
    Serial.println(temperatureF);

    // Turn servo only for more than 2° delta
    diff = damper - oldDamper;
    if (abs(diff) > 2) {
      lcd.print(" x");
      delay(500);
      myservo.attach(10);
      if (diff > 2) {
        for (int i = 0; i < diff; i++) {
            angle = (oldDamper + i + 1) * servoAngle / (maxDamper * servoCalibration) - servoOffset;
            myservo.write(angle);
            delay(200);
          }
      }
      
      if (diff < 2) {
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
    delay(5000);
  }
}
