/* Zoumobot Interupt and Boot mode extensions
Written by Grant Hilgert
January 2018

This extensions created a development mode and competiton mode for the zoumobot.
The mode is selected via an external DIP switch.

In development mode, the device behaves like a normal arduinio, allowing code to be uploaded over the serial port.
In compention mode, code can still be uplaoded over the serial port. However, the correct IR signal must be received 
before execution is allowed to begin

Using guide at https://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/
and the Atmel ATmega169A/PA/329A/PA/3290A/PA/649A/P/6490A/P datasheet


//For 
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
#include <inttypes.h>
#include "zoumobot_extension.h"



//Performs setup functions for extension
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

//Enables IR detectors
void zoumobot_enable(void){
  //Reenable all interupts
  sei();
}

//Not sure how im going to start the compeition yet
void zoumobot_start(void){


}
//Reads DIP switch and sets the boot mode
void zoumobot_set_mode(void){  

  /*/Rules: PE2 == HIGH; DIP0 is set to off
            PE2 == LOW;  DIP0 is set to on
  /*/
  //Reads and mask DIP0 - Mode selection bit 
  zoumobot_temp = PINE & 0b00000100
    if(zoumobot_temp = 0){
    /*/----Competition Mode----/*/
      //First, turn off Development led off
      PORTA &= 0b11011111;
      //Now, turn on Competition led 
      PORTA |= 0b00010000;
      //Set Development mode bit so the program will wait for IR input later on
      zoumobot_mode = 0;
    }
  
  /*/----Development Mode----/*/
    else{
      //Competition led off
      PORTA &= 0b11101111;
      //Development LED on
      PORTA |= 0b00100000
      //Set Development mode bit so that the IR detectors will be disabled
      zoumobot_mode = 1;

  }
}

//Terminate program
void zoumobot_shutdown(void){
  //Clear PWM Timers
  //Set leds
  //Wait for infinity
    while(true);
}

//Interrupt Routine 

/*ISR(PCINT0_vect){
  //Insert code check in IR input is valid
  
  
  //Shutdowns the zoumobot
  zoumobot_shutdown();
  */
}
