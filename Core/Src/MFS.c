/*
 * MFS.c
 *
 *  Created on: Mar 22, 2023
 *      Author: chaselewis
 */
#include "MFS.h"

//change later so it returns whether or not the button is pressed
int check_btn1(){
	if (HAL_GPIO_ReadPin(SHLD_A1_GPIO_Port, SHLD_A1_Pin)==GPIO_PIN_RESET) return 1;
	return 0;
}


//change later so it returns whether or not the button is pressed
int check_btn2(){
	if (HAL_GPIO_ReadPin(SHLD_A2_GPIO_Port, SHLD_A2_Pin)==GPIO_PIN_RESET) return 1;
	return 0;
}

//change later so it returns whether or not the button is pressed
int check_btn3(){
	if (HAL_GPIO_ReadPin(SHLD_A3_GPIO_Port, SHLD_A3_Pin)==GPIO_PIN_RESET) return 1;
	return 0;
}

void shiftOut(uint8_t val)
{
	for(int ii=0x80; ii; ii>>=1) {
		HAL_GPIO_WritePin(SHLD_D7_SEG7_Clock_GPIO_Port, SHLD_D7_SEG7_Clock_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SHLD_D8_SEG7_Data_GPIO_Port, SHLD_D8_SEG7_Data_Pin, (val&ii)!=0);
		HAL_GPIO_WritePin(SHLD_D7_SEG7_Clock_GPIO_Port, SHLD_D7_SEG7_Clock_Pin, GPIO_PIN_SET);
	}
}

void WriteNumberToSegment(int Segment, int Value)
{
  HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_RESET);
  shiftOut(SEGMENT_MAP[Value]);
  shiftOut(SEGMENT_SELECT[Segment] );
  HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_SET);
}



void displayNum(int count){

	int dig1 = count / 1000;					//divide the number to get the thousands place
	int dig2 = (count - (1000 * dig1)) / 100;	//get the hundreds place
	int dig3 = (count - (100 * dig2)) / 10;		//get the tens places
	int dig4 = count % 10;						//get the ones place

	// Display the digits
	//none zeros with get displayed successively
	if (dig1 != 0){
		HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_RESET);
		shiftOut(SEGMENT_MAP[dig1]);
		shiftOut(SEGMENT_SELECT[0]);
		HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_SET);
		osDelay(100);
	}
	if (dig2 != 0){
		HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_RESET);
		shiftOut(SEGMENT_MAP[dig2]);
		shiftOut(SEGMENT_SELECT[1]);
		HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_SET);
		osDelay(100);
	}
	if (dig3 != 0){
		HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_RESET);
		shiftOut(SEGMENT_MAP[dig3]);
		shiftOut(SEGMENT_SELECT[2]);
		HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_SET);
		osDelay(100);
	}
	HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_RESET);
	shiftOut(SEGMENT_MAP[dig4]);
	shiftOut(SEGMENT_SELECT[3]);
	HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_SET);
	osDelay(100);

}


