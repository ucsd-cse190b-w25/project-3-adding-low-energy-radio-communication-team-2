/*
 * i2c.c
 *
 *  Created on: Feb 3, 2025
 *      Author: akamdar
 */


/* Include memory map of our MCU */
#include <stm32l475xx.h>
#include <stdio.h>

void i2c_init()
{
	// Enable clock for GPIOB
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    // Configure PB10 (SCL) and PB11 (SDA) as alternate function mode (AF4 for I2C2)
    GPIOB->MODER &= ~(GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk);
    // Set to Alternate Function mode
    GPIOB->MODER |= (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1);

    // Configure open-drain, high-speed,and enable pull-up
    GPIOB->OTYPER |= (GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED11);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD10_Msk | GPIO_PUPDR_PUPD11_Msk);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0);

    // Set AF4 (I2C2) for PB10 and PB11
    GPIOB->AFR[1] &= ~((0xF << GPIO_AFRH_AFSEL10_Pos) | (0xF << GPIO_AFRH_AFSEL11_Pos));
    GPIOB->AFR[1] |= ((4 << GPIO_AFRH_AFSEL10_Pos) | (4 << GPIO_AFRH_AFSEL11_Pos));

    // Enable the I2C clock by stopping it, setting the timer, and enabling.
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
    I2C2->CR1 &= ~I2C_CR1_PE;
    // Timing set to a frequency of 250 KHz.
    I2C2->TIMINGR = (0 << 28) | (0x3 << 0) | (0x3 << 8) | (0x3 << 16) | (0x3 << 20);
    I2C2->CR1 |= I2C_CR1_PE;
}

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len) {
    // Write
	if (dir == 0) {
        I2C2->CR2 = (address << 1) | (0 << 10) | (len << 16) | I2C_CR2_START; // Write mode

        // For each byte of data (the first would be the address on the peripheral), write the data.
        for (uint8_t i = 0; i < len; i++) {
        	// Wait for TXIS flag. If NACK during this operation, return an error.
			while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {
				if (I2C2->ISR & I2C_ISR_NACKF) {
					// Clear NACKF and send STOP because operation failed.
					I2C2->ICR |= I2C_ICR_NACKCF;
					I2C2->CR2 |= I2C_CR2_STOP;
					return 2;
				}
			}
			// Write the data.
			I2C2->TXDR = data[i];
		}
        // Wait for the transfer complete flag.
		while (!(I2C2->ISR & I2C_ISR_TC));
    }
    // Read
    else {
        // Send register address by writing one byte (first byte is assumed to be the address).
        I2C2->CR2 = (address << 1) | (0 << 10) | (1 << 16) | I2C_CR2_START;

        while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {
            if (I2C2->ISR & I2C_ISR_NACKF) {
                I2C2->ICR |= I2C_ICR_NACKCF;
                I2C2->CR2 |= I2C_CR2_STOP;
                return 2;
            }
        }
        // Send the register address
        I2C2->TXDR = data[0];
        // Wait for transfer complete
        while (!(I2C2->ISR & I2C_ISR_TC));

        // Read `len` bytes from the data received. This uses MMIO.
        I2C2->CR2 = (address << 1) | (1 << 10) | (len << 16) | I2C_CR2_START;

        for (uint8_t i = 0; i < len; i++) {
        	// Wait for RXNE flag.
            while (!(I2C2->ISR & I2C_ISR_RXNE));
            data[i] = I2C2->RXDR;

            // Send NACK before last byte to signal that reading is complete.
            if (i == len - 1) {
                I2C2->CR2 |= I2C_CR2_NACK;
            }
        }
    }

	// Set stop, wait until stop is detected, then clear the flag.
	I2C2->CR2 |= I2C_CR2_STOP;
	while (!(I2C2->ISR & I2C_ISR_STOPF));
	I2C2->ICR |= I2C_ICR_STOPCF;

    return 0;
}

