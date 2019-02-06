/*
 * fxos_interaction.h
 *
 *  Created on: 24 Jan 2019
 *      Author: jacob
 */

#ifndef FXOS_INTERACTION_H_

#define FXOS_INTERACTION_H_

#include "fsl_common.h"
#include <fsl_fxos.h>

typedef struct accel_raw_data {
	int16_t x;
	int16_t y;
	int16_t z;
} accel_raw_data_t;

typedef enum sensor_range {
	range2 = 0,
	range4 = 1,
	range8 = 1 << 1
} sensor_range_t;

/**
 * Sets up and initializes the fxosHandle for communication with acceleremoter over I2C
 */
void Fxi_init(fxos_handle_t * fxosHandle);

/**
 * Gets the current speed of the chip. Not velocity
 */
int32_t Fxi_getVelocity(fxos_handle_t * fxosHandle);

/**
 * Gives you the raw accelerometer data in the 3 x, y, and z axes.
 */
void Fxi_getRawAccelData(fxos_handle_t * fxosHandle, accel_raw_data_t * accelData);

/**
 * Gets the accelerometer data converted to m/s. Takes into consideration
 * the current accelerometer sensor range
 */
void Fxi_getCalculatedAccelData(fxos_handle_t * fxosHandle, accel_raw_data_t * accelData);

/**
 * Sets the data range of the accelerometer.
 * With a greater range you get lesser resolution of data
 */
void Fxi_setSensorRange(fxos_handle_t * fxosHandle, sensor_range_t range);

#endif /* FXOS_INTERACTION_H_ */
