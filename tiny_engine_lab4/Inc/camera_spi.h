/*
 * camera_spi.h
 *
 *  Created on: 2020/1/3
 *      Author: wmchen
 */

#ifndef CAMERA_SPI_H_
#define CAMERA_SPI_H_

#include "stm32746g_discovery.h"

/* Define for CS control */
#define ARDUCAM_CS_PORT GPIOI
#define ARDUCAM_CS_PIN GPIO_PIN_0
#define RESET GPIO_PIN_RESET
#define SET GPIO_PIN_SET
#define ARDUCAM_SPI_TIMEOUT 100
#define ARDUCHIP_TEST1  0x00  //TEST register

#define ARDUCAM_CS_LOW HAL_GPIO_WritePin(ARDUCAM_CS_PORT, ARDUCAM_CS_PIN, RESET);
#define ARDUCAM_CS_HIGH HAL_GPIO_WritePin(ARDUCAM_CS_PORT, ARDUCAM_CS_PIN, SET);
#if !defined(ARDUCAM_CS_PORT) || !defined(ARDUCAM_CS_PIN) || !defined(ARDUCAM_CS_LOW) || !defined(ARDUCAM_CS_HIGH)
#error please define Arducam SPI settings
#endif

/* functions */
int camSPISetup();
void camSPIReset();
void camWriteReg(const uint8_t reg, const uint8_t val);
int camSPIErrorTest();
void camCoreDump();
uint8_t camTransfer(const uint8_t val);
uint8_t camTransfers(uint8_t *buf, const uint32_t length);
uint8_t camReadReg(const uint8_t reg);
uint8_t camReadRegBit(uint8_t addr, uint8_t bit);
int camSPIErrorTest();

/* Definition for SPIx clock resources */
#define SPIx                             SPI2
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOI_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI2_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI2_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_1
#define SPIx_SCK_GPIO_PORT               GPIOI
#define SPIx_SCK_AF                      GPIO_AF5_SPI2
#define SPIx_MISO_PIN                    GPIO_PIN_14
#define SPIx_MISO_GPIO_PORT              GPIOB
#define SPIx_MISO_AF                     GPIO_AF5_SPI2
#define SPIx_MOSI_PIN                    GPIO_PIN_15
#define SPIx_MOSI_GPIO_PORT              GPIOB
#define SPIx_MOSI_AF                     GPIO_AF5_SPI2

/* Define for CS control */
#define ARDUCAM_CS_PORT GPIOI
#define ARDUCAM_CS_PIN GPIO_PIN_0
#define RESET GPIO_PIN_RESET
#define SET GPIO_PIN_SET
#define ARDUCAM_SPI_TIMEOUT 100
#define ARDUCHIP_TEST1  0x00  //TEST register

#endif /* CAMERA_SPI_H_ */
