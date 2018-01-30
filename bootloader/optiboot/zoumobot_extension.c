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

#define DIP0 PE2
#define DIP1 PE3
#define IR0 PE0
#define IR1 PE7

*/
#include <avr/interrupt.h>



//Temp Variable used for reading DIP0
uint8_t zoumobot_temp = 0;
uint8_t zoumobot_dev_mode = 1;

//Setup
void zoumobot_setup(void){
  //temporarily disabled all Interupts so we can saftly modify them
  cli():
  //Sets DIP0, DIP1, IR0 and IR1 as inputs, leaves SDA,SCL,RX,TX alone
  DDRE &= 0b00110011;
  //Sets up built in LED's as outputs
  DDRA |= 0b11110000;
  //Enabled PCINT0 through PCINT7
  EIMSK |= 0b00010000;
  //Mask so that PCINT6 & PCINT7 are the only two interupts enabled.
  PCMSK0 |= 0b11000000;
  
}

void zoumobot_enable(void){
  //Reenable all interupts
  sei();
}
void zoumobot_start(void){


}
void zoumobot_set_mode(void){
  
//Read DIP0 - Mode selection bit

zoumobot_temp = PINE & 0b00000100

 //----Competition Mode----
if(zoumobot_temp = 0){
  //Set Development led off
  PORTA &= 0b11011111;
  //Set Competition led on
  PORTA |= 0b00010000;
//Enables interrupts 
 zoumobot_enable();
//disabled dev moce 
zoumobot_dev_mode = 0;
  }
  
  //----Development Mode----
else{
  //Competition led off
  PORTA &= 0b11101111;
  //Development LED on
  PORTA |= 0b00100000
//enable dev mode
zoumobot_dev_mode = 1;
  }
}

//Post match or Emergency Stop
void zoumobot_shutdown(void){
  //Clear PWM Timers
  
  //Set leds
  
  while(true);
}

//Interrupt Routine 
ISR(PCINT0_vect){
  //Insert code check in IR input is valid
  
  
  //Shutdowns the zoumobot
  zoumobot_shutdown();
}
