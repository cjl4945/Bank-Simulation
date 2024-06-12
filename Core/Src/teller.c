/*
 * teller.c
 *
 *  Created on: Mar 23, 2023
 *      Author: chaselewis
 */


#include "teller.h"
#include "customer.h"
#include "string.h"
#include "stdio.h"

TELLER tellers[NUM_TELLERS];

int teller_init(int num){
	//Create each teller
	for (int i = 0; i<NUM_TELLERS;i++){
		TELLER *t = &tellers[i];
		memset(t, 0, sizeof(TELLER));
		t->customers_served = 0;
		t->wait_start = 0;
		t->max_wait_time = 0;
		t->total_wait_time = 0;
		t->max_transaction_time = 0 ;
		t->total_transaction_time = 0;
		t->num_breaks = 0;
		t->total_break_time = 0;
		t->max_break_time = TELLER_BREAK_MIN;
		t->min_break_time = TELLER_BREAK_MAX;
		t->break_set = 0;
		t->status = AVAIL;
	}
	return 0;
}

int teller_end(int num){
	if (customer_queue.len != 0){return 0;}
	else{
		for (int ii = 0; ii < NUM_TELLERS; ii++){
			TELLER t = tellers[ii];
			if (t.status != AVAIL){
				//failure, all tellers aren't done
				return 0;
			}
		}
	}
	//Success, All tellers are done
	return 1;
}

char* get_status_name(enum teller_status status){
	switch(status)
	{
		case AVAIL: return"AVAIL";
		case BUSY: return "BUSY";
		case BREAK: return "BREAK";
		default: return "BUSY";
	}

}
