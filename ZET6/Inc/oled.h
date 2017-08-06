#ifndef __OLED_H
#define __OLED_H	

#include "stm32f1xx_hal.h"
#include "stdint.h"	    


//---------------------------OLED�˿ڶ���--------------------------  					   

#define OLED_SCL_HIGH()	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET)//D0
#define OLED_SCL_LOW()	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET)

#define OLED_SDA_HIGH()	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET)//D1
#define OLED_SDA_LOW()	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET)

#define OLED_RST_HIGH()	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET)//RST
#define OLED_RST_LOW()	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET)

#define OLED_DC_HIGH()	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET)	//DC
#define OLED_DC_LOW()		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET)


#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

void OLED_WR_Byte(uint8_t dat, uint8_t cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);  		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *p, uint8_t size);	 
#endif  
	 







 

