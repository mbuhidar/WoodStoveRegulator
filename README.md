# WoodStoveRegulator v2.2 - Arduino Powered Stove Damper Regulation

 System drives an Arduino microcontroller-based device that automatically 
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

