#ifndef __esp8266_H
#define __esp8266_H

#include "at32f403a_407_board.h"

uint8_t esp8266_Init(void);
void Esp8266_send_data(u8 *buf, u16 len);
void esp8266_sub_getparms(void);
void esp8266_pub_data(char* pubparm);

uint8_t flTstr_FirpTwo(float data, char* datTostr);
float strTof_FirpTwo(char* strTodat);

#endif
