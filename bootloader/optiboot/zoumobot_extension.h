#include <inttypes.h>


#ifndef zoumobot_extension_H_
#define zoumobot_extension_H_



//Temp Variable used for reading DIP0
uint8_t zoumobot_temp = 0;
//Varialbe to hold boot mode
uint8_t zoumobot_mode = 1;

void zoumobot_setup(void);
void zoumobot_enable(void);
void zoumobot_start(void);
void zoumobot_set_mode(void);
void zoumobot_shutdown(void);

#endif