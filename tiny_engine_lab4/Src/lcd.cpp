/*
 * lcd.cpp
 *
 *  Created on: 2020/1/3/
 *      Author: wmchen
 */

#include "lcd.h"
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"

#define TRANS 128 //0~255: transparency

void loadRGB565LCD(uint32_t x, uint32_t y, uint32_t width, uint32_t height, uint16_t * src, uint8_t resize)
{
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			//fill in resize x resize tile: simple resizing now
			uint16_t color = src[i*width+j];
//			BSP_LCD_DrawPixel(x + j, y + i, color);
			for(int ti = 0; ti < resize; ti++){
				for(int tj = 0; tj < resize; tj++){
					BSP_LCD_DrawPixel(x + j*resize + tj, y + i*resize + ti, color);
				}
			}
		}
	}

}

void drawRedBackground(int x1, int x2, int y1, int y2)
{
	uint16_t red = 63488;//11111,000000,00000

	for (int i = x1-1; i < x2; i++)
		for (int j = y1 - 1; j < y2; j++) {
			BSP_LCD_DrawPixel(i, j, red);
		}
}

void drawGreenBackground(int x1, int x2, int y1, int y2)
{
	uint16_t green = 2016;//00000,111111,00000

	for (int i = x1 - 1; i < x2; i++)
		for (int j = y1 - 1; j < y2; j++) {
			BSP_LCD_DrawPixel(i, j, green);
		}
}

void drawBlueBackground(int x1, int x2, int y1, int y2)
{
	uint16_t blue = 2016 + 63488;//11111,000000,00000

	for (int i = x1-1; i < x2; i++)
		for (int j = y1 - 1; j < y2; j++) {
			BSP_LCD_DrawPixel(i, j, blue);
		}
}

void drawBlackBackground(int x1, int x2, int y1, int y2)
{
	uint16_t black = 0;//00000,111111,00000

	for (int i = x1 - 1; i < x2; i++)
		for (int j = y1 - 1; j < y2; j++) {
			BSP_LCD_DrawPixel(i, j, black);
		}
}


/*
 *
 * */
void displaystring(char* buf, int x, int y){
	BSP_LCD_DisplayStringAt(x, y, buf, LEFT_MODE);
}

#define USENEW
int unper_cnt = 0;
/*
 *
 * */
#ifdef USENEW
void detectResponse(int person, float ms, int training_mode, int pred, int label){
	char buf[20];
	if(person){
		unper_cnt = 0;
		if (training_mode){
			drawGreenBackground(270,480,40,100);
			drawGreenBackground(270,480,125,180);
			drawGreenBackground(270,480,205,250);
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			sprintf(buf," Prediction:");
			BSP_LCD_DisplayStringAt(273, 80, buf, LEFT_MODE);
			sprintf(buf,"  class %d  ", pred);
			BSP_LCD_DisplayStringAt(273, 100, buf, LEFT_MODE);
			sprintf(buf,"Ground True:");
			BSP_LCD_DisplayStringAt(273, 120, buf, LEFT_MODE);
			sprintf(buf,"  class %d   ", label);
			BSP_LCD_DisplayStringAt(273, 140, buf, LEFT_MODE);
		}
		else{
			drawBlueBackground(270,480,40,100);
			drawBlueBackground(270,480,125,180);
			drawBlueBackground(270,480,205,250);
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			BSP_LCD_DisplayStringAt(273, 100, "   Person   ", LEFT_MODE);
		}
	}
	else{
		if (training_mode){
			drawRedBackground(270,480,40,100);
			drawRedBackground(270,480,125,180);
			drawRedBackground(270,480,205,250);
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			sprintf(buf," Prediction:");
			BSP_LCD_DisplayStringAt(273, 80, buf, LEFT_MODE);
			sprintf(buf,"  class %d   ", pred);
			BSP_LCD_DisplayStringAt(273, 100, buf, LEFT_MODE);
			sprintf(buf,"Ground-Truth");
			BSP_LCD_DisplayStringAt(273, 120, buf, LEFT_MODE);
			sprintf(buf,"  class %d   ", label);
			BSP_LCD_DisplayStringAt(273, 140, buf, LEFT_MODE);
		}
		else{
			drawBlackBackground(270,480,40,100);
			drawBlackBackground(270,480,125,180);
			drawBlackBackground(270,480,205,250);
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			BSP_LCD_DisplayStringAt(273, 100, "  No Person ", LEFT_MODE);
		}
	}

	if (ms == 0)
		return;
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    volatile float rate = 1000/ms;
    volatile int decimal = (int)rate;
    volatile int floating = (int)((rate - (float)decimal)*1000);
    sprintf(buf, "  fps:%d.%03d ", decimal, floating);
	BSP_LCD_DisplayStringAt(273, 180, buf, LEFT_MODE);
}

#else
void detectResponse(int person, float ms){
	if(person){
		unper_cnt = 0;
		drawRedBackground(270,480,40,100);
		drawRedBackground(270,480,125,180);
		drawRedBackground(270,480,205,250);
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(273, 100, "  No Person ", LEFT_MODE);
	}
	else{
		unper_cnt++;
//		if(unper_cnt > 2){
			drawGreenBackground(270,480,40,100);
			drawGreenBackground(270,480,125,180);
			drawGreenBackground(270,480,205,250);
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			BSP_LCD_DisplayStringAt(273, 100, "   Person   ", LEFT_MODE);
//		}
	}

    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    char buf[20];
    volatile float rate = 1000/ms;
    volatile int decimal = (int)rate;
    volatile int floating = (int)((rate - (float)decimal)*1000);
    sprintf(buf, "  fps:%d.%03d ", decimal, floating);
	BSP_LCD_DisplayStringAt(273, 180, buf, LEFT_MODE);
}
#endif


/*
 * This function set up the lcd as the initial content
 * */
void lcdsetup(){
    RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

    /* LCD clock configuration */
    /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
    /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz */
    /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/5 = 38.4 Mhz */
    /* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_4 = 38.4/4 = 9.6Mhz */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
    PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
    PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* Configure LCD : Only one layer is used */
    BSP_LCD_Init();

    /* LCD Initialization */
	BSP_LCD_LayerRgb565Init(0, LCD_FB_START_ADDRESS);
	BSP_LCD_LayerRgb565Init(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));
//    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
//    BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));

    /* Enable the LCD */
    BSP_LCD_DisplayOn();

    /* Select the LCD Background Layer  */
    BSP_LCD_SelectLayer(0);

    /* Clear the Background Layer */
    BSP_LCD_Clear(LCD_COLOR_BLACK);

    /* Select the LCD Foreground Layer  */
    BSP_LCD_SelectLayer(1);

    /* Clear the Foreground Layer */
    BSP_LCD_Clear(LCD_COLOR_BLACK);

    /* Configure the transparency for foreground and background :
    Increase the transparency */
    BSP_LCD_SetTransparency(0, 0);
    BSP_LCD_SetTransparency(1, 100);

    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
}


