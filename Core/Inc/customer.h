/*
 * customer.h
 *
 *  Created on: Mar 23, 2023
 *      Author: chaselewis
 */

#ifndef INC_CUSTOMER_H_
#define INC_CUSTOMER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "cmsis_os.h"

#define MAX_CUSTOMERS (50)

#define CUSTOMER_ARRIVAL_MIN (60)
#define CUSTOMER_ARRIVAL_MAX (240)
#define RAND_CUSTOMER_TIME (random_distance(CUSTOMER_ARRIVAL_MIN, CUSTOMER_ARRIVAL_MAX))


typedef struct{
	uint32_t enter_time;
} CUSTOMER;

typedef struct {
	int len;
	int max_len;
	int max_wait_time;
	int total_wait_time;
	int num_served;
	TaskHandle_t handle;
} CUSTOMER_QUEUE;

extern CUSTOMER_QUEUE customer_queue;

int customer_queue_init();
int customer_queue_pop();
int customer_queue_push();


#endif /* INC_CUSTOMER_H_ */
