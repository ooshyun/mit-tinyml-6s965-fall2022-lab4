/*
 * lcd.h
 *
 *  Created on: 2020/1/3/
 *      Author: wmchen
 */

#ifndef LCD_H_
#define LCD_H_
#include <stdio.h>

//#include "stm32746g_discovery.h"
//#include "stm32746g_discovery_lcd.h"

void loadRGB565LCD(uint32_t x, uint32_t y, uint32_t width, uint32_t height, uint16_t * src, uint8_t resize);
void loadgrayscale2LCD(uint8_t * src);
void lcdsetup();

#define USENEW
#ifdef USENEW
void detectResponse(int person, float ms, int training_mode, int pred, int label);// int show_fps, int show_train_option);
#else
void detectResponse(int person, float ms);// int show_fps, int show_train_option);
#endif

void displaystring(char* buf, int x, int y);

#endif /* LCD_H_ */
