/* Zoumobot Interupt and Boot mode extensions
Written by Grant Hilgert

This extensions created a development mode and competiton mode for the zoumobot.
The mode is selected via an external DIP switch.

In development mode, the device behaves like a normal arduinio, allowing code to be uploaded over the serial port.
In compention mode, code can still be uplaoded over the serial port. However, the correct IR signal must be received 
before execution is allowed to begin

Using guide at https://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/
DIP0 PCINT 2
DIP1 PCINT 3

IR0 PCINT 6
IR1 PCINT 7

*/
#include <avr/interrupt.h>

#define DIP0 PE2
#define DIP1 PE3
#define IR0 PE0
#define IR1 PE7

//Setup
void zoumobot_setup(void){
  //temporarily disabled all Interupts so we can saftly modify them
  cli():
  //Pin Change Interrupt Enable 0
  //Enabled PCINT0 through PCINT7
  EIMSK |= 0b00010000;
  //Mask so that PCINT6 & PCINT7 are the only two interupts enabled.
  PCMSK0 |= 0b11000000;
  
}

void zoumobot_enable(void){
  //Reenable all interupts
  sei();
}
void zoumobot_check_mode(void){
  
}

//Post match or Emergency Stop
void zoumobot_shutdown(void){
  //Clear PWM Timers
  
  //Set leds
  
  while(true);
}

ISR(PCINT0_vect){
  //Insert code to 
  
  
  //Shutdowns the zoumobot
  zoumobot_shutdown();
}
