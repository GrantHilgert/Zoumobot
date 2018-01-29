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
