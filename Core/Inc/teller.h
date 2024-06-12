/*
 * teller.h
 *
 *  Created on: Mar 23, 2023
 *      Author: chaselewis
 */

#ifndef INC_TELLER_H_
#define INC_TELLER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#define NUM_TELLERS 3	// Maximum 8

#define TELLER_TRANSACT_MIN		(30)	//seconds
#define TELLER_TRANSACT_MAX		(480)	//seconds
#define NEXT_BREAK__MIN		(1800)	//seconds
#define NEXT_BREAK__MAX		(3600)	//seconds
#define TELLER_BREAK_MIN (60) 	//seconds
#define TELLER_BREAK_MAX (240)	//seconds
#define RAND_TELLER_TIME	(random_distance(TELLER_TRANSACT_MIN, TELLER_TRANSACT_MAX))
#define RAND_NEXT_BREAK		(random_distance(NEXT_BREAK__MIN, NEXT_BREAK__MAX))
#define RAND_BREAK_LENGTH	(random_distance(TELLER_BREAK_MIN, TELLER_BREAK_MAX))

enum teller_status{AVAIL, BUSY, BREAK};

typedef struct {
	// some parameters to track
	int customers_served;
	uint32_t wait_start;
	int max_wait_time;
	uint32_t total_wait_time;
	int max_transaction_time;
	uint32_t total_transaction_time;
	uint32_t next_break;
	int num_breaks;
	int total_break_time;
	int max_break_time;
	int min_break_time;
	int break_set;
	enum teller_status status;
} TELLER;

extern TELLER tellers[NUM_TELLERS];

int teller_init(int num);

int teller_end(int num);

char* get_status_name(enum teller_status status);


#endif /* INC_TELLER_H_ */
