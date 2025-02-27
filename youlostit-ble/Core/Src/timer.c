/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{
	// Enable register 1 for peripheral clock APB1, so that read/write access is supported.
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	// Disable the counter for the timer so that it is safe to make changes.
	timer->CR1 &= ~TIM_CR1_CEN;
	// Reset the counters to 0
	timer->CNT = 0;
	// Trigger an update event for the registers and reinitialize the counter.
	timer->EGR = TIM_EGR_UG;
	// Set prescaler to 3 so that the frequency is set to CK_CNT = (4 MHz)/(3+1) = 1 MHz.
	timer->PSC = 3;
	// Set auto-reload to 999 so that the frequency is set to (1 MHz)/(999+1) = 1KHz
	// This equates to a period of 1 ms for auto-reloading, so an interrupt is triggered
	// at some multiple of 1ms later defined.
	timer->ARR = 999;
	// Enable auto-reload preload. This ensures that changes to ARR only
	// take place upon completion of a cycle.
	timer->CR1 |= TIM_CR1_ARPE;
	// Enable timer's internal interrupt.
	// This ensures that interrupts are registered as events.
	timer->DIER |= TIM_DIER_UIE;
	// Enable interrupt controller's interrupt
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 2);
	// Enable the counter for the timer so that it begins.
	timer->CR1 |= TIM_CR1_CEN;
}

void timer_reset(TIM_TypeDef* timer)
{
	// Reset the timer's counters to 0.
	timer->CNT = 0;
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
	// Disable the counters for safe changes.
    timer->CR1 &= ~TIM_CR1_CEN;
    /* Set the ARR value to reflect the new period. Since the current frequency is 1 MHz
     * (defined through prescale ratio), we define the ARR value as such so that the equation
     * equates to freq = (10^6)/(1000*period_ms) = 10^3/period_ms Hz.
     * The period of this is then 1/freq = period_ms/10^3 which is period_ms milliseconds, as desired.
     */
    timer->ARR = (1000*period_ms) - 1;
    // Trigger an update event for the registers and reinitialize the counter.
    timer->EGR |= TIM_EGR_UG;
    // Enable the timer.
    timer->CR1 |= TIM_CR1_CEN;
}
