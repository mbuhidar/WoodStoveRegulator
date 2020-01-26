# WoodStoveRegulator

 System drives an Arduino microcontroller based device that auomatically 
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
