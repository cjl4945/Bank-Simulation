/*
 * MFS.h
 *
 *  Created on: Mar 22, 2023
 *      Author: chaselewis
 */

#ifndef INC_MFS_H_
#define INC_MFS_H_

#include "main.h"




int check_btn1();
int check_btn2();
int check_btn3();
void shiftOut(uint8_t val);
void WriteNumberToSegment(int Segment, int Value);
void displayNum(int count);




#endif /* INC_MFS_H_ */
