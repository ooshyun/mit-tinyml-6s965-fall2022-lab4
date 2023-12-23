/*
 * camera.h
 *
 *  Created on: 2020/1/9
 *      Author: wmchen
 */

#include <stdio.h>
#include <assert.h>
#include "camera.h"
#include "camera_spi.h"
#include "camera_i2c.h"
#include "ov2640_regs.h"
#include <JPEGDecoder.h>

uint8_t imgBuf[5120];
uint32_t imgLength;
bool is_header = false;
static byte m_fmt = JPEG;

//************************************************Private functions
/*
 *
 * */
void flush_fifo(void)
{
    camWriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

/*
 *
 * */
void clear_fifo_flag(void )
{
    camWriteReg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

/*
 *
 * */
void start_capture(void)
{
    camWriteReg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

//************************************************Public functions

/*
 * set format of camera output
 * */
void set_format(byte fmt){
    m_fmt = fmt;
}

/*
 * init camera return error times
 * */
int initCamera()
{
    int camrror = camSPISetup();
    camrror += camI2CSetup();

    wrSensorReg8_8(0xff, 0x01);
    //set as slave device
    wrSensorReg8_8(0x12, 0x80);

    HAL_Delay(100);
    if (m_fmt == JPEG)
    {
        wrSensorRegs8_8(OV2640_JPEG_INIT);
        wrSensorRegs8_8(OV2640_YUV422);
        wrSensorRegs8_8(OV2640_JPEG);
        wrSensorReg8_8(0xff, 0x01);
        wrSensorReg8_8(0x15, 0x00);
        wrSensorRegs8_8(OV2640_320x240_JPEG);
        HAL_Delay(100);
        //wrSensorReg8_8(0xff, 0x00);
        //wrSensorReg8_8(0x44, 0x32);
    }
    else
    {
        wrSensorRegs8_8(OV2640_QVGA);
    }

    OV2640_set_JPEG_size(OV2640_160x120);

    HAL_Delay(100);
    return camrror;
}

void OV2640_set_JPEG_size(uint8_t size)
{
  switch(size)
  {
    case OV2640_160x120:
      wrSensorRegs8_8(OV2640_160x120_JPEG);
      break;
    case OV2640_176x144:
      wrSensorRegs8_8(OV2640_176x144_JPEG);
      break;
    case OV2640_320x240:
      wrSensorRegs8_8(OV2640_320x240_JPEG);
      break;
    case OV2640_352x288:
      wrSensorRegs8_8(OV2640_352x288_JPEG);
      break;
    case OV2640_640x480:
      wrSensorRegs8_8(OV2640_640x480_JPEG);
      break;
    case OV2640_800x600:
      wrSensorRegs8_8(OV2640_800x600_JPEG);
      break;
    case OV2640_1024x768:
      wrSensorRegs8_8(OV2640_1024x768_JPEG);
      break;
    case OV2640_1280x1024:
      wrSensorRegs8_8(OV2640_1280x1024_JPEG);
      break;
    case OV2640_1600x1200:
      wrSensorRegs8_8(OV2640_1600x1200_JPEG);
      break;
    default:
      wrSensorRegs8_8(OV2640_320x240_JPEG);
      break;
  }
}

/*
 *
 * */
uint32_t read_fifo_length(void)
{
    uint32_t len1,len2,len3,length=0;
    len1 = camReadReg(FIFO_SIZE1);
    len2 = camReadReg(FIFO_SIZE2);
    len3 = camReadReg(FIFO_SIZE3) & 0x7f;
    length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
    return length;
}

uint32_t getbufSize()
{
    return imgLength;
}

uint8_t getbufChar(int i)
{
    return imgBuf[i];
}


/*
 *
 * */
int DecodeandProcessRGB565(int image_width, int image_height, uint16_t* image_data)
{
	// Parse the JPEG headers. The image will be decoded as a sequence of Minimum
	// Coded Units (MCUs), which are 16x8 blocks of pixels.
	JpegDec.decodeArray(imgBuf, imgLength);

	// Crop the image by keeping a certain number of MCUs in each dimension
	const int keep_x_mcus = image_width / JpegDec.MCUWidth;
	const int keep_y_mcus = image_height / JpegDec.MCUHeight;

	// Calculate how many MCUs we will throw away on the x axis
	const int skip_x_mcus = JpegDec.MCUSPerRow - keep_x_mcus;
	// Roughly center the crop by skipping half the throwaway MCUs at the
	// beginning of each row
	const int skip_start_x_mcus = skip_x_mcus / 2;
	// Index where we will start throwing away MCUs after the data
	const int skip_end_x_mcu_index = skip_start_x_mcus + keep_x_mcus;
	// Same approach for the columns
	const int skip_y_mcus = JpegDec.MCUSPerCol - keep_y_mcus;
	const int skip_start_y_mcus = skip_y_mcus / 2;
	const int skip_end_y_mcu_index = skip_start_y_mcus + keep_y_mcus;

	// Pointer to the current pixel
	uint16_t* pImg;
	// Color of the current pixel
	uint16_t color;

	// Loop over the MCUs
	while (JpegDec.read()) {
	// Skip over the initial set of rows
	if (JpegDec.MCUy < skip_start_y_mcus) {
	  continue;
	}
	// Skip if we're on a column that we don't want
	if (JpegDec.MCUx < skip_start_x_mcus ||
		JpegDec.MCUx >= skip_end_x_mcu_index) {
	  continue;
	}
	// Skip if we've got all the rows we want
	if (JpegDec.MCUy >= skip_end_y_mcu_index) {
	  continue;
	}
	// Pointer to the current pixel
	pImg = JpegDec.pImage;

	// The x and y indexes of the current MCU, ignoring the MCUs we skip
	int relative_mcu_x = JpegDec.MCUx - skip_start_x_mcus;
	int relative_mcu_y = JpegDec.MCUy - skip_start_y_mcus;

	// The coordinates of the top left of this MCU when applied to the output
	// image
	int x_origin = relative_mcu_x * JpegDec.MCUWidth;
	int y_origin = relative_mcu_y * JpegDec.MCUHeight;

	// Loop through the MCU's rows and columns
	for (int mcu_row = 0; mcu_row < JpegDec.MCUHeight; mcu_row++) {
	  // The y coordinate of this pixel in the output index
	  int current_y = y_origin + mcu_row;
	  for (int mcu_col = 0; mcu_col < JpegDec.MCUWidth; mcu_col++) {
		// Read the color of the pixel as 16-bit integer
		color = *pImg++;

		// The x coordinate of this pixel in the output image
		int current_x = x_origin + mcu_col;
		// The index of this pixel in our flat output buffer
		int index = (current_y * image_width) + current_x;
		image_data[index] = color;
	  }
	}
	}
}


int DecodeandProcessAndRGB(int image_width, int image_height, int8_t* image_data, uint16_t* lcd_data, int scale_factor)
{
	// Parse the JPEG headers. The image will be decoded as a sequence of Minimum
	// Coded Units (MCUs), which are 16x8 blocks of pixels.
	JpegDec.decodeArray(imgBuf, imgLength);

	// Crop the image by keeping a certain number of MCUs in each dimension
	const int keep_x_mcus = image_width / JpegDec.MCUWidth;
	const int keep_y_mcus = image_height / JpegDec.MCUHeight;

	// Calculate how many MCUs we will throw away on the x axis
	const int skip_x_mcus = JpegDec.MCUSPerRow - keep_x_mcus;
	// Roughly center the crop by skipping half the throwaway MCUs at the
	// beginning of each row
	const int skip_start_x_mcus = skip_x_mcus / 2;
	// Index where we will start throwing away MCUs after the data
	const int skip_end_x_mcu_index = skip_start_x_mcus + keep_x_mcus;
	// Same approach for the columns
	const int skip_y_mcus = JpegDec.MCUSPerCol - keep_y_mcus;
	const int skip_start_y_mcus = skip_y_mcus / 2;
	const int skip_end_y_mcu_index = skip_start_y_mcus + keep_y_mcus;

	// Pointer to the current pixel
	uint16_t* pImg;
	// Color of the current pixel
	uint16_t color;

	for(int i = 0; i < (image_height / scale_factor) * (image_width / scale_factor) * 3; i++)
		image_data[i] = -128;

	// Loop over the MCUs
	while (JpegDec.read()) {
	// Skip over the initial set of rows
	if (JpegDec.MCUy < skip_start_y_mcus) {
	  continue;
	}
	// Skip if we're on a column that we don't want
	if (JpegDec.MCUx < skip_start_x_mcus ||
		JpegDec.MCUx >= skip_end_x_mcu_index) {
	  continue;
	}
	// Skip if we've got all the rows we want
	if (JpegDec.MCUy >= skip_end_y_mcu_index) {
	  continue;
	}
	// Pointer to the current pixel
	pImg = JpegDec.pImage;

	// The x and y indexes of the current MCU, ignoring the MCUs we skip
	int relative_mcu_x = JpegDec.MCUx - skip_start_x_mcus;
	int relative_mcu_y = JpegDec.MCUy - skip_start_y_mcus;

	// The coordinates of the top left of this MCU when applied to the output
	// image
	int x_origin = relative_mcu_x * JpegDec.MCUWidth;
	int y_origin = relative_mcu_y * JpegDec.MCUHeight;

	// Loop through the MCU's rows and columns
	for (int mcu_row = 0; mcu_row < JpegDec.MCUHeight; mcu_row++) {
	  // The y coordinate of this pixel in the output index
	  int current_y = y_origin + mcu_row;
	  for (int mcu_col = 0; mcu_col < JpegDec.MCUWidth; mcu_col++) {
		// Read the color of the pixel as 16-bit integer
		color = *pImg++;
		// Extract the color values (5 red bits, 6 green, 5 blue)
		uint8_t r, g, b;
		r = ((color & 0xF800) >> 11) * 8;
		g = ((color & 0x07E0) >> 5) * 4;
		b = ((color & 0x001F) >> 0) * 8;
		// Convert to grayscale by calculating luminance
		// See https://en.wikipedia.org/wiki/Grayscale for magic numbers
//		float gray_value = (0.2126 * r) + (0.7152 * g) + (0.0722 * b);

		// The x coordinate of this pixel in the output image
		int current_x = x_origin + mcu_col;
		// The index of this pixel in our flat output buffer
		int index = (current_y * image_width) + current_x;
		int a_index = index*3;

		if(current_y >= 120)
			continue;

		lcd_data[index] = color;
		if(scale_factor == 1){
			image_data[index*3] = r-128;
			image_data[index*3+1] = g-128;
			image_data[index*3+2] = b-128;
			lcd_data[index] = color;
		}
		if(scale_factor != 1 && (current_y % scale_factor != 0 || current_x % scale_factor != 0))
			continue;//skip this pixel
		//boundary case
		if(image_width % scale_factor != 0)
			continue;
		int width = image_width;
		if(width > 120)
			width = 120;
		int Iindex = (current_y / scale_factor) * (image_width / scale_factor ) + current_x / scale_factor ;

		image_data[Iindex*3] = r-128;
		image_data[Iindex*3+1] = g-128;
		image_data[Iindex*3+2] = b-128;
	  }
	}
	}
}

/*
 *
 * */
int DecodeandProcess(int image_width, int image_height, uint8_t* image_data)
{
	// Parse the JPEG headers. The image will be decoded as a sequence of Minimum
	// Coded Units (MCUs), which are 16x8 blocks of pixels.
	JpegDec.decodeArray(imgBuf, imgLength);

	// Crop the image by keeping a certain number of MCUs in each dimension
	const int keep_x_mcus = image_width / JpegDec.MCUWidth;
	const int keep_y_mcus = image_height / JpegDec.MCUHeight;

	// Calculate how many MCUs we will throw away on the x axis
	const int skip_x_mcus = JpegDec.MCUSPerRow - keep_x_mcus;
	// Roughly center the crop by skipping half the throwaway MCUs at the
	// beginning of each row
	const int skip_start_x_mcus = skip_x_mcus / 2;
	// Index where we will start throwing away MCUs after the data
	const int skip_end_x_mcu_index = skip_start_x_mcus + keep_x_mcus;
	// Same approach for the columns
	const int skip_y_mcus = JpegDec.MCUSPerCol - keep_y_mcus;
	const int skip_start_y_mcus = skip_y_mcus / 2;
	const int skip_end_y_mcu_index = skip_start_y_mcus + keep_y_mcus;

	// Pointer to the current pixel
	uint16_t* pImg;
	// Color of the current pixel
	uint16_t color;

	// Loop over the MCUs
	while (JpegDec.read()) {
	// Skip over the initial set of rows
	if (JpegDec.MCUy < skip_start_y_mcus) {
	  continue;
	}
	// Skip if we're on a column that we don't want
	if (JpegDec.MCUx < skip_start_x_mcus ||
		JpegDec.MCUx >= skip_end_x_mcu_index) {
	  continue;
	}
	// Skip if we've got all the rows we want
	if (JpegDec.MCUy >= skip_end_y_mcu_index) {
	  continue;
	}
	// Pointer to the current pixel
	pImg = JpegDec.pImage;

	// The x and y indexes of the current MCU, ignoring the MCUs we skip
	int relative_mcu_x = JpegDec.MCUx - skip_start_x_mcus;
	int relative_mcu_y = JpegDec.MCUy - skip_start_y_mcus;

	// The coordinates of the top left of this MCU when applied to the output
	// image
	int x_origin = relative_mcu_x * JpegDec.MCUWidth;
	int y_origin = relative_mcu_y * JpegDec.MCUHeight;

	// Loop through the MCU's rows and columns
	for (int mcu_row = 0; mcu_row < JpegDec.MCUHeight; mcu_row++) {
	  // The y coordinate of this pixel in the output index
	  int current_y = y_origin + mcu_row;
	  for (int mcu_col = 0; mcu_col < JpegDec.MCUWidth; mcu_col++) {
		// Read the color of the pixel as 16-bit integer
		color = *pImg++;
		// Extract the color values (5 red bits, 6 green, 5 blue)
		uint8_t r, g, b;
		r = ((color & 0xF800) >> 11) * 8;
		g = ((color & 0x07E0) >> 5) * 4;
		b = ((color & 0x001F) >> 0) * 8;
		// Convert to grayscale by calculating luminance
		// See https://en.wikipedia.org/wiki/Grayscale for magic numbers
		float gray_value = (0.2126 * r) + (0.7152 * g) + (0.0722 * b);

		// The x coordinate of this pixel in the output image
		int current_x = x_origin + mcu_col;
		// The index of this pixel in our flat output buffer
		int index = (current_y * image_width) + current_x;
		image_data[index] = static_cast<uint8_t>(gray_value);
	  }
	}
	}
}

/*
 *
 * */
int PerformCapture() {
    // Make sure the buffer is emptied before each capture
    flush_fifo();
    HAL_Delay(1);
    clear_fifo_flag();
    HAL_Delay(1);
    // Start capture
    start_capture();

    // Wait for indication that it is done
    while (!camReadRegBit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    }

    // Clear the capture done flag
    clear_fifo_flag();
    read_fifo_burst();

    return 0;
}


/*
 *
 * */
int StartCapture() {
    // Make sure the buffer is emptied before each capture
    flush_fifo();
    HAL_Delay(1);
    clear_fifo_flag();
    HAL_Delay(1);
    // Start capture
    start_capture();

    return 0;
}

/*
 *
 * */
int ReadCapture() {
    // Wait for indication that it is done
    while (!camReadRegBit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    }

    // Clear the capture done flag
    clear_fifo_flag();
    read_fifo_burst();

    return 0;
}


uint8_t read_fifo_burst()
{
    uint32_t length = read_fifo_length();
    if (length >= MAX_FIFO_SIZE) //512 kb
    {
//        Serial.println(F("ACK CMD Over size. END"));
        return 0;
    }
    if (length == 0 ) //0 kb
    {
//        Serial.println(F("ACK CMD Size is 0. END"));
        return 0;
    }
    ARDUCAM_CS_LOW;
    volatile uint8_t burst = camTransfer(BURST_FIFO_READ);//Set fifo burst mode
    int index;

    camTransfers(imgBuf, length);

    //find ture file length
    for (index = length-1; index >= 0; index--) {
        if(imgBuf[index] != 0){
            break;
        }
    }

    imgLength = index + 1;
//    HAL_Delay(1);
    ARDUCAM_CS_HIGH;

    is_header = false;
    return 1;
}

