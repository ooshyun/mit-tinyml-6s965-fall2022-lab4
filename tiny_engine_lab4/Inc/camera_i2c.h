/*
 * camera_i2c.h
 *
 *  Created on: 2020/1/4
 *      Author: wmchen
 */

#ifndef CAMERA_I2C_H_
#define CAMERA_I2C_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"


/* Exported types ------------------------------------------------------------*/
struct sensor_reg {
  uint8_t reg;
  uint8_t val;
};

const struct sensor_reg testRegs[]  =
{
    {0xff, 0x0},
    {0x2c, 0xff},
};

/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor I2Cx/I2Cx instance used and associated
   resources */
/* Definition for I2Cx clock resources */
#define I2Cx                            I2C1
#define RCC_PERIPHCLK_I2Cx              RCC_PERIPHCLK_I2C1
#define RCC_I2CxCLKSOURCE_SYSCLK        RCC_I2C1CLKSOURCE_PCLK1
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_8
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_9
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C1

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

/* functions */
int camI2CSetup();
int camI2CErrorTest();
//void i2cReset();
int wrSensorReg16_8( uint16_t regID, uint16_t regDat);
int wrSensorRegs16_8(const struct sensor_reg reglist[]);
int rdSensorReg16_8(uint16_t regID, uint8_t* regDat);
int wrSensorReg8_8(uint8_t regID, uint8_t regDat);
int wrSensorRegs8_8(const struct sensor_reg reglist[]);
int rdSensorReg8_8(uint8_t regID, uint8_t* regDat);

#endif /* CAMERA_I2C_H_ */
