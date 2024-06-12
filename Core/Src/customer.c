/*
 * customer.c
 *
 *  Created on: Mar 23, 2023
 *      Author: chaselewis
 */

#include "customer.h"
#include "string.h"
#include "stdio.h"

CUSTOMER_QUEUE customer_queue;

static void customer_task(void *params);
static QueueHandle_t queue_handle;


int  customer_queue_init(){
	queue_handle = xQueueCreate(MAX_CUSTOMERS, sizeof(CUSTOMER));
	assert(queue_handle != NULL);
	//initialize all metrics of the queue to zero to start
	customer_queue.len = 0;
	customer_queue.max_len = 0;
	customer_queue.max_wait_time = 0;
	customer_queue.total_wait_time = 0;
	customer_queue.num_served = 0;
	TaskHandle_t th;

	BaseType_t err = xTaskCreate(customer_task, "CustomerTask", 128, NULL, 24, &th);
	assert(err == pdPASS);
	customer_queue.handle = th;
	return 0;
}

static void customer_task(void *params){
	while(1){
		osDelay(SIM_SEC_TO_US(RAND_CUSTOMER_TIME)/1000);

		CUSTOMER c = {htim2.Instance->CNT};

		int err = customer_queue_push(&c);
		assert(!err);
	}
}

int customer_queue_push(CUSTOMER *customer) {
	if (customer_queue.len < MAX_CUSTOMERS) {
		// Add the customer to the queue.
		BaseType_t err = xQueueSendToBack(queue_handle, customer, portMAX_DELAY);
		assert(err = pdPASS);
		// Update the queue metrics.
		osMutexAcquire(customerQueueMutexHandle, portMAX_DELAY);
		int len = ++customer_queue.len;
		if (len > customer_queue.max_len) {
			customer_queue.max_len = len;
		}
		osMutexRelease(customerQueueMutexHandle);
		// Success.
		return 0;
	} else {
		// Failure-- queue is full.
		return 1;
	}
}

int customer_queue_pop() {
	if (customer_queue.len > 0) {
		// Remove the customer from the queue.
		CUSTOMER c;
		BaseType_t err = xQueueReceive(queue_handle, &c, portMAX_DELAY);
		assert(err = pdPASS);
		// Update the queue metrics.
		osMutexAcquire(customerQueueMutexHandle, portMAX_DELAY);
		customer_queue.len--;
		uint32_t wait_time = htim2.Instance->CNT - c.enter_time;
		wait_time = US_TO_SIM_SEC(wait_time);
		customer_queue.total_wait_time += wait_time;
		if (wait_time > customer_queue.max_wait_time) {
			customer_queue.max_wait_time = wait_time;
		}
		customer_queue.num_served++;
		osMutexRelease(customerQueueMutexHandle);
		// Success.
		return 0;
	} else {
		// Failure-- queue is empty.
		return 1;
	}
}
