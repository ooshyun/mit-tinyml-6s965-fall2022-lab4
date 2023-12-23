/*
 * camera_spi.c
 *
 *  Created on: 2020/1/3
 *      Author: wmchen
 */


#include "camera_spi.h"

struct sensor_reg {
  uint8_t reg;
  uint8_t val;
};

uint8_t core[][2] =
{
    {0x00, 0x00},//Test
    {0x01, 0x00},//Capture control
    {0x03, 0x00},//Timing
    {0x04, 0x00},//FIFO control
    {0x05, 0x00},//Test mode bit[0] = CAM, 1 = Test Data
    {0x06, 0x00},//GPIO write reg, bit[0]: reset IO value, Bit[1]: Sensor power down IO  Bit[2]: Sensor power enable I/O
    {0x07, 0x00},//Bit[7] write 1 to reset CLPLD
    {0x3C, 0x00},//Burst FIFO read
    {0x3D, 0x00},//Single FIFO read
    {0x40, 0x00},//Arduchip version [7:4] integer of revision number, [3:0]: decimal of revision number
    {0x41, 0x00},//[0] vsync pin, [1]trigger status, [3]: camera write FIFO done flag
    {0x42, 0x00},//camera write FIFO size[7:0]
    {0x43, 0x00},//camera write FIFO size[15:8]
    {0x44, 0x00},//camera write FIFO size[26:16]
    {0x45, 0x00},//FIFO status reg, [0]: 0 = unfull, 1 = full
    {0x46, 0x00},//Arduchip version year [6:0]
    {0x47, 0x00},//Arduchip version month [3:0]
    {0x48, 0x00},//Arduchip version date [4:0]
    {0xFF, 0xFF},//end of core
};

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

//************************************************Private functions

void delayUS(uint32_t us) {
    #define CLOCK_CYCLE_PER_INSTRUCTION    1
    #define CLOCK_FREQ                      216  //IN MHZ (e.g. 16 for 16 MHZ)

    volatile int cycleCount = us * CLOCK_FREQ / CLOCK_CYCLE_PER_INSTRUCTION;

    while (cycleCount--);
    // 1uS is elapsed :-)
    // Sure?
    // :-/
}

/*
 * set corresponding bit
 * */
void setBit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = camReadReg(addr);
  camWriteReg(addr, temp | bit);
}

/*
 * clear corresponding bit
 * */
void clearBit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = camReadReg(addr);
  camWriteReg(addr, temp & (~bit));
}

/*
 * get corresponding bit status
 * */
uint8_t getBit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = camReadReg(addr);
  temp = temp & bit;
  return temp;
}

//************************************************Public functions

static SPI_HandleTypeDef SpiHandle;

/*
 * PIN Connection
 * SPI2: MISO->PB14(D12). MOSI->PB15(D11). SCK->PI_1(D13). CS(NSS)->PI_0(D5). VCC-> 3.3V. GND->GND
 * I2C1: SCL->PB8(D15). SDA->PB9(D14)
 *
 * Setting: Mode 0 CPOL=0, CPHA=0
 * The ArduCAM SPI slave interface is fixed SPI mode 0 with POL = 0 and PHA = 1. The maximum speed of SCLK is designed for 8MHz, care should taken do not over clock the maximum 8MHz.
 *
 * Possible issues:?
[1]https://community.st.com/s/question/0D50X00009Xkdwu/spi-stm32f7
Hello, I found where is the mistake. The schematic of stm32f746-DISCOVERY  board is wrong. PI0 is to output ARD_D5 and PA8 is to output ARD_D10.
*/

/*
 * setup SPI2 for our Arducam
 * return: error count
 * */
int camSPISetup()
{
    int error = 0;

    /* Set the SPI parameters */
    SpiHandle.Instance               = SPIx;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.Mode = SPI_MODE_MASTER;

    if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
    {
        /* Initialization Error */
        error++;
    }

    /* SPI CS GPIO pin configuration  */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin       = GPIO_PIN_0;
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    /* reset camera SPI and test */
    camSPIReset();
    error += camSPIErrorTest();
    return error;
}

/*
 * write and read the arducam's register to test the SPI port
 * */
int camSPIErrorTest()
{
    uint8_t val = 0x55;
    volatile uint8_t rval;
    int error = 0;

    //trials: write 0 as 0x55 ...
    for(int i = 0; i < 10; i++){
        val++;
        camWriteReg(0x00, val);
        rval = camReadReg(0x00);
        if(rval != val)
            error++;
    }

    return error;
}

/*
 * reset the camera via the established SPI
 * */
void camSPIReset()
{
    uint8_t cmd1[2] = {0x07 , 0x80};
    uint8_t cmd2[2] = {0x07 , 0x00};

    camWriteReg(cmd1[0], cmd1[1]);
    HAL_Delay(100);
    camWriteReg(cmd2[0], cmd2[1]);
    HAL_Delay(100);
}

/*
 * write "val" to "reg"
 * */
void camWriteReg(const uint8_t reg, const uint8_t val)
{
    uint8_t buff[2] = {reg | 0x80, val};
    ARDUCAM_CS_LOW;
    HAL_SPI_Transmit(&SpiHandle, (uint8_t*)&buff, 2, 100);
    ARDUCAM_CS_HIGH;
    HAL_Delay(1);
}

/*
 * write "val" to "reg"
 * Note: used for burst read , self-control CS is needded
 * */
uint8_t camTransfer(const uint8_t val)
{
    uint8_t ret;
    HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)&val, (uint8_t*)&ret, 1, 100);
//    delayUS(15);
    return ret;
}

#define DummyLength 8096
#define DummyVal 0x00
uint8_t dummy[DummyLength];
static int dummyinit = 0;
/*
 * write "val" to "reg"
 * Note: used for burst read , self-control CS is needded
 * */
uint8_t camTransfers(uint8_t *buf, const uint32_t length)
{
    if(!dummyinit){
        for(int i = 0; i < DummyLength; i++)
            dummy[i] = DummyVal;
        dummyinit = 1;
    }
    uint8_t ret;
    HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)dummy, (uint8_t*)buf, length, 100);
//    delayUS(15);
    return ret;
}

/*
 * read the register's value
 * */
uint8_t camReadReg(const uint8_t reg)
{
    uint8_t buff[2] = {reg, 0x00}; //dummy
    uint8_t rbuff[2];
    ARDUCAM_CS_LOW;
    HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)&buff, (uint8_t*)&rbuff, 2, 100);
    ARDUCAM_CS_HIGH;
//    HAL_Delay(1);
    return rbuff[1];
}

/*
 * get the register's bit
 * */
uint8_t camReadRegBit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = camReadReg(addr);
  temp = temp & bit;
  return temp;
}

#define DEBUG
/*
 * get regs of the arcuchip's core
 * */
void camCoreDump()
{
#ifdef DEBUG
    int index = 0;
    while(core[index][0] != 0xFF)
    {
        core[index][1] = camReadReg(core[index][0]);
        index++;
    }
#endif
}
