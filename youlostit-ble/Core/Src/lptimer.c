/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "lptimer.h"


void lptimer_init()
{
	RCC->CIER |= RCC_CIER_LSIRDYIE;
    // Clear LSIRDYF flag by setting LSIRDYC (bit 23 of RCC_CSR)
    RCC->CICR |= RCC_CICR_LSIRDYC;
    RCC->CSR |= RCC_CSR_LSION;
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);
    while ((RCC->CIFR & RCC_CIFR_LSIRDYF) == 0);

    // Select LSI as the clock source for LPTIM1
    RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;
    RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0;  // 01: LSI clock

    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;

    LPTIM1->CR &= ~LPTIM_CR_ENABLE;

    LPTIM1->CFGR &= ~LPTIM_CFGR_CKSEL;
    LPTIM1->CFGR &= ~LPTIM_CFGR_TRIGEN;
    LPTIM1->CFGR &= ~LPTIM_CFGR_COUNTMODE;

    // Set prescaler (available values: 1, 2, 4, 8, 16, 32, etc.)
    LPTIM1->CFGR &= ~LPTIM_CFGR_PRESC;  // Clear existing prescaler value
    LPTIM1->CFGR |= LPTIM_CFGR_PRESC_1;  // Set prescaler to 4 (equivalent to 8kHz clock)

    // Enable interrupt
    LPTIM1->IER |= LPTIM_IER_ARRMIE;
    NVIC_EnableIRQ(LPTIM1_IRQn);
    NVIC_SetPriority(LPTIM1_IRQn, 1);

    LPTIM1->IER |= LPTIM_IER_ARROKIE;

    LPTIM1->CR |= LPTIM_CR_ENABLE;
    __NOP();
    __NOP();

    // Set Auto-reload value for 1ms interrupt (ARR = (8kHz * 1ms) - 1 = 7)
    LPTIM1->ARR = 7999;

    // Start LPTIM in continuous mode
    LPTIM1->CR |= LPTIM_CR_CNTSTRT;
}

void lptimer_reset()
{
	LPTIM1->ICR |= LPTIM_ICR_ARRMCF;  // Clear flag
	LPTIM1->CNT = 0;                  // Reset counter
}

void lptimer_set_ms(uint16_t period_ms)
{
  LPTIM1->ARR = ((8000 * period_ms) / 1000) - 1;
}
