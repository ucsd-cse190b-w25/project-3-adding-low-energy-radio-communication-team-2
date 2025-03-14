/*
 * timer.h
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#ifndef LPTIMER_H_
#define LPTIMER_H_

/* Include the type definitions for the timer peripheral */
#include <stm32l475xx.h>

void lptimer_init();
void lptimer_reset();
void lptimer_set_ms(uint16_t period_ms);

#endif /* TIMER_H_ */
