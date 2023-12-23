/*
 * camera.h
 *
 *  Created on: 2020/1/9
 *      Author: wmchen
 */
#ifndef CAMERA_H_
#define CAMERA_H_

#define byte uint8_t
#define BMP       0
#define JPEG      1

#define OV2640_160x120    0 //160x120
#define OV2640_176x144    1 //176x144
#define OV2640_320x240    2 //320x240
#define OV2640_352x288    3 //352x288
#define OV2640_640x480    4 //640x480
#define OV2640_800x600    5 //800x600
#define OV2640_1024x768   6 //1024x768
#define OV2640_1280x1024  7 //1280x1024
#define OV2640_1600x1200  8 //1600x1200

#define ARDUCHIP_FIFO         0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK       0x01
#define FIFO_START_MASK       0x02
#define FIFO_RDPTR_RST_MASK     0x10
#define FIFO_WRPTR_RST_MASK     0x20

#define ARDUCHIP_TRIG         0x41  //Trigger source
#define CAP_DONE_MASK         0x08

#define BURST_FIFO_READ     0x3C  //Burst FIFO read operation
#define FIFO_SIZE1        0x42  //Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2        0x43  //Camera write FIFO size[15:8]
#define FIFO_SIZE3        0x44  //Camera write FIFO size[18:16]

#define MAX_FIFO_SIZE   0x5FFFF     //384KByte

uint8_t getbufChar(int i);
uint32_t getbufSize();
int initCamera();
uint8_t read_fifo_burst();
void OV2640_set_JPEG_size(uint8_t size);
int PerformCapture();
int ReadCapture();
int StartCapture();
int DecodeandProcessAndRGB(int image_width, int image_height, int8_t* image_data, uint16_t* lcd_data, int scale_factor);
int DecodeandProcess(int image_width, int image_height, uint8_t* image_data);
int DecodeandProcessRGB565(int image_width, int image_height, uint16_t* image_data);

#endif /* CAMERA_H_ */
