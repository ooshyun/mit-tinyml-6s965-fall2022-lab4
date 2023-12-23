/*
 * camera_i2c.cpp
 *
 *  Created on: 2020/1/4
 *      Author: wmchen
 */

#include "camera_i2c.h"
#include "stm32f7xx_hal.h"

#define I2C_ADDRESS        0x0F
#define CAM_WRITE_ADDRESS  0x60
#define CAM_READ_ADDRESS   0x61
#define SSCB_TIMEOUT 10

#define OV2640_CHIPID_HIGH  0x0A
#define OV2640_CHIPID_LOW   0x0B
#define OV2640_MAX_FIFO_SIZE    0x5FFFF     //384KByte

/* I2C TIMING Register define when I2C clock source is APB1 (SYSCLK/4) */
/* I2C TIMING is calculated in case of the I2C Clock source is the APB1CLK = 50 MHz */
/* This example use TIMING to 0x40912732 to reach 100 kHz speed (Rise time = 700 ns, Fall time = 100 ns) */
#define I2C_TIMING      0x40912732


void delayUS(int us) {
    #define CLOCK_CYCLE_PER_INSTRUCTION    1
    #define CLOCK_FREQ                      2160  //IN MHZ (e.g. 16 for 16 MHZ)

    volatile int cycleCount = us * CLOCK_FREQ / CLOCK_CYCLE_PER_INSTRUCTION;

    while (cycleCount--);
    // 1uS is elapsed :-)
    // Sure?
    // :-/
}


/*
 * set up the i2c
 * */
static I2C_HandleTypeDef I2cHandle;
int camI2CSetup()
{
    int error = 0;

    /* Set the I2C parameters */
    I2cHandle.Instance             = I2Cx;
    I2cHandle.Init.Timing          = I2C_TIMING;
    I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
    I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle.Init.OwnAddress2     = 0xFF;
    I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
    {
        /* Initialization Error */
        error++;
    }

    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(&I2cHandle,I2C_ANALOGFILTER_ENABLE);

    error += camI2CErrorTest();
    return error;
}

/*
 * test the i2c
 * */
int camI2CErrorTest()
{
    int error = 0;
    uint8_t vid, pid;

    //Check if the camera module type is OV2640
    wrSensorReg8_8(0xff, 0x01);
    rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))){
        error++;
    }

    return error;
}

//*********************the following functions are from github: Arducam/STM32
/**
 * This function will help you for setting register values on camera sensor
 * @param address Target camera sscb bus device address
 * @param regID Target camera sensor register address
 * @param regDat Target camera sensor register value
 * @return is SSCB bus communication is ok?
 */
int wrSensorReg16_8(uint16_t regID, uint16_t regDat)
{
    uint8_t tx_buffer[3];
    tx_buffer[0] = regID>>8;
    tx_buffer[1] = (uint8_t)regID;
    tx_buffer[2] = (uint8_t)regDat;

    if(HAL_I2C_Master_Transmit(&I2cHandle, CAM_WRITE_ADDRESS, (uint8_t*)tx_buffer, sizeof(tx_buffer), SSCB_TIMEOUT) != HAL_OK)
        return false;

    HAL_Delay(1);

    return true;
}

/**
 * This function will send all sensor camera settings struct's to camera sensor over
 * I2C or SSCB bus
 * @param reglist target camera settings struct
 * @return I2C or SSCB bus error size
 */
int wrSensorRegs16_8(const struct sensor_reg reglist[])
{
  int err = 0;

  unsigned int reg_addr;
  unsigned char reg_val;
  const struct sensor_reg *next = reglist;

  while ((reg_addr != 0xffff) | (reg_val != 0xff))
  {
    reg_addr =next->reg;
    reg_val = next->val;

    if(reg_addr == 0xffff)
      break;

    err = wrSensorReg16_8(reg_addr, reg_val);
    next++;
  }
  return err;
}

/**
 * This function will help you for reading register values on camera sensor
 * @param address Target camera sscb bus device address
 * @param regID Target camera sensor register address
 * @param regDat Target camera sensor register value
 * @return is SSCB bus communication is ok?
 */
int rdSensorReg16_8(uint16_t regID, uint8_t* regDat)
{
    uint8_t tx_buffer[2];
    tx_buffer[0] = regID>>8;
    tx_buffer[1] = (uint8_t)regID;

    if(HAL_I2C_Master_Transmit(&I2cHandle, CAM_WRITE_ADDRESS , (uint8_t*)tx_buffer, sizeof(tx_buffer), SSCB_TIMEOUT) != HAL_OK)
        return false;

    HAL_Delay(1);

    if(HAL_I2C_Master_Receive(&I2cHandle, CAM_READ_ADDRESS, (uint8_t*)regDat, 1,  SSCB_TIMEOUT) != HAL_OK)
        return false;

    HAL_Delay(1);

    return true;
}

//***************************************The following code is based on Arducam/STM32
#define TEST
int wrSensorReg8_8(uint8_t regID, uint8_t regDat)
{
    uint8_t tx_buffer[2];
    tx_buffer[0] = regID & 0xFF;
    tx_buffer[1] = regDat & 0xFF;
    HAL_I2C_Master_Transmit(&I2cHandle, CAM_WRITE_ADDRESS, (uint8_t*)tx_buffer, sizeof(tx_buffer), SSCB_TIMEOUT);

    HAL_Delay(1);

    return 1;
}


int wrSensorRegs8_8(const struct sensor_reg reglist[])
{
    const struct sensor_reg *next = reglist;
    uint8_t reg_addr = 0;
    uint8_t reg_val = 0;
    while ((reg_addr != 0xff) | (reg_val != 0xff))
    {
      reg_addr = next->reg;
      reg_val = next->val;
      wrSensorReg8_8(reg_addr, reg_val);
      next++;
    }

    return 1;
}


int rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
{
    if(HAL_I2C_Master_Transmit(&I2cHandle, CAM_WRITE_ADDRESS, &regID, 1, SSCB_TIMEOUT) != HAL_OK)
      return false;

    HAL_Delay(1);

    if(HAL_I2C_Master_Receive(&I2cHandle, CAM_READ_ADDRESS, (uint8_t*)regDat, 1,  SSCB_TIMEOUT) != HAL_OK)
      return false;

    HAL_Delay(1);
    return 1;
}

