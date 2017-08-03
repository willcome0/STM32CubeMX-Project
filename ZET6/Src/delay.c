#include "delay.h"
#include "stm32f1xx_hal.h"


/*************************************************************************
*函数名：  下面这3个
*函数功能：自用延时函数的初始化； ms延时； us延时；
*入口参数：time： 对应单位填入即可
*返回值：  无
*作者：    康滢
*时间：		 2017.8.2
*备注：	   主要就是修改系统定时器的中断周期，使其一次中断为1us。
					 修改完后HAL库自带的延时函数延时单位就变成了1us。
					 (·\ 。/·)且此时us延时函数也能带入大数量级
**************************************************************************/
void Delay_Config(void)
{
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000);
}

void Delay_ms(uint32_t time)
{
	HAL_Delay(time*1000);
}
void Delay_us(uint32_t time)
{
	HAL_Delay(time);
}

