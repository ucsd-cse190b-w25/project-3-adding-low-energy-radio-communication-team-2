/*
 * lsm6dsl.c
 *
 *  Created on: Feb 5, 2025
 *      Author: akamdar
 */


/* Include memory map of our MCU */
#include <stm32l475xx.h>
#include <stdio.h>
#include "i2c.h"

// Address of accelerometer.
#define ACC_ADDR 0x6A

void lsm6dsl_init()
{
	// Write 0x60 to CTRL1_XL at 0x10.
	uint8_t ctrl1_xl_data[] = {0x10, 0x60};
	uint8_t transaction = i2c_transaction(ACC_ADDR, 0, ctrl1_xl_data, 2);
	if (transaction > 0) {return;}

	// Write 0x01 to INT1_CTRL at 0x0d.
	uint8_t int1_ctrl_data[] = {0x0d, 0x01};
	transaction = i2c_transaction(ACC_ADDR, 0, int1_ctrl_data, 2);
	if (transaction > 0) {return;}
}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {
	// Initialize addr_or_data to be the byte that contains the address of the register, then the data read from the register.
	uint8_t addr_or_data[1];
	uint8_t t;
	// Reading x first:
	*addr_or_data = 0x28; // Address of x_low
	t = i2c_transaction(ACC_ADDR, 1, addr_or_data, 1);
	if (t > 0) {return;}
	*x = (int16_t)(addr_or_data[0]);

	*addr_or_data = 0x29; // Address of x_high
	t = i2c_transaction(ACC_ADDR, 1, addr_or_data, 1);
	if (t > 0) {return;}
	*x |= (int16_t)(addr_or_data[0] << 8);


	// Reading y next:
	*addr_or_data = 0x2A; // Address of y_low
	t = i2c_transaction(ACC_ADDR, 1, addr_or_data, 1);
	if (t > 0) {return;}
	*y = (int16_t)(addr_or_data[0]);

	*addr_or_data = 0x2B; // Address of y_high
	t = i2c_transaction(ACC_ADDR, 1, addr_or_data, 1);
	if (t > 0) {return;}
	*y |= (int16_t)(addr_or_data[0] << 8);


	// Reading z next:
	*addr_or_data = 0x2C; // Address of z_low
	t = i2c_transaction(ACC_ADDR, 1, addr_or_data, 1);
	if (t > 0) {return;}
	*z = (int16_t)(addr_or_data[0]);

	*addr_or_data = 0x2D; // Address of z_high
	t = i2c_transaction(ACC_ADDR, 1, addr_or_data, 1);
	if (t > 0) {return;}
	*z |= (int16_t)(addr_or_data[0] << 8);

}
