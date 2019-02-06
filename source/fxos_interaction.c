/*
#include <fsl_fxos.h>
 * fxos_interaction.c
 *
 *  Created on: 24 Jan 2019
 *      Author: jacob
 */

#include "fxos_interaction.h"

#include "board.h"

#include "fsl_i2c.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "math.h"

#include "pin_mux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* The TPM instance/channel used for board */
#define BOARD_TIMER_BASEADDR TPM2
#define BOARD_FIRST_TIMER_CHANNEL 0U
#define BOARD_SECOND_TIMER_CHANNEL 1U
/* Get source clock for TPM driver */
#define BOARD_TIMER_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_Osc0ErClk)
#define TIMER_CLOCK_MODE 1U
/* I2C source clock */
#define ACCEL_I2C_CLK_SRC I2C1_CLK_SRC
#define I2C_BAUDRATE 100000U

#define I2C_RELEASE_SDA_PORT PORTC
#define I2C_RELEASE_SCL_PORT PORTC
#define I2C_RELEASE_SDA_GPIO GPIOC
#define I2C_RELEASE_SDA_PIN 3U
#define I2C_RELEASE_SCL_GPIO GPIOC
#define I2C_RELEASE_SCL_PIN 2U
#define I2C_RELEASE_BUS_COUNT 100U

/*******************************************************************************
 * Variables
 ******************************************************************************/
i2c_master_handle_t g_MasterHandle;
/* FXOS device address */
const uint8_t g_accel_address = 0x1FU;

/*******************************************************************************
 * Code
 ******************************************************************************/
static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

/*  */
void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}

void Fxi_init(fxos_handle_t * fxosHandle){
	    i2c_master_config_t i2cConfig;
	    uint32_t i2cSourceClock;

	    /* Board pin, clock, debug console init */
//	    BOARD_BootClockRUN();
	    BOARD_I2C_ReleaseBus();
	    BOARD_InitI2C();

	    i2cSourceClock = CLOCK_GetFreq(ACCEL_I2C_CLK_SRC);
	    fxosHandle->base = BOARD_ACCEL_I2C_BASEADDR;
	    fxosHandle->i2cHandle = &g_MasterHandle;

	    I2C_MasterGetDefaultConfig(&i2cConfig);
	    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &i2cConfig, i2cSourceClock);
	    I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_MasterHandle, NULL, NULL);

	    fxosHandle->xfer.slaveAddress = g_accel_address;
//	    (FXOS_ReadReg(&fxosHandle, WHO_AM_I_REG, &regResult, 1) != kStatus_Success)

	    FXOS_Init(fxosHandle);
}

int32_t Fxi_getSpeed(fxos_handle_t * fxosHandle){
	/**
	 * Rough steps:
	 * 1. Convert raw data to multiples of g
	 * 2. Subtract 1 g vertically to account for gravity
	 * 3. Convert to velocity. Integration is involved
	 */

		return 1;
}

void Fxi_getRawAccelData(fxos_handle_t * fxosHandle, accel_raw_data_t * accelData){
		fxos_data_t sensorData;

		FXOS_ReadSensorData(fxosHandle, &sensorData);

		/* Get the X and Y data from the sensor data structure in 14 bit left format data
		 *
		 * Here we convert 14 bit data into 16 bit
		 * */

		accelData->x = ((int16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB);
		accelData->y = ((int16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB);
		accelData->z = ((int16_t)((uint16_t)sensorData.accelZMSB << 8) | (uint16_t)sensorData.accelZLSB);
}

void Fxi_getCalculatedAccelData(fxos_handle_t * fxosHandle, accel_raw_data_t * accelData){

	uint8_t sensorRange = 0;

	Fxi_getRawAccelData(fxosHandle, accelData);
	FXOS_ReadReg(fxosHandle, XYZ_DATA_CFG_REG, &sensorRange, 1);

}

void Fxi_setSensorRange(fxos_handle_t * fxosHandle, sensor_range_t range){
	//TODO: Doesn't currently seem to do anything, needs investigation.

	FXOS_WriteReg(fxosHandle, XYZ_DATA_CFG_REG, range);

}
