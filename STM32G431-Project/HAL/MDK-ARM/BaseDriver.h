#ifndef BASE_DRIVER_H
#define BASE_DRIVER_H

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

//UART
#include "usart.h"
inline void EchoUart(void* huart){
	unsigned char ch = 0;
	if( !HAL_UART_Receive(huart,&ch,1,0) ){
		HAL_UART_Transmit(huart,&ch,1,100);
	}
}

#define PrintfUart(huart,fmt,...) do{\
	static char buff[1024] = "";\
	sprintf(buff,fmt,__VA_ARGS__);\
	HAL_UART_Transmit(&huart,(unsigned char*)buff,strlen(buff),100);\
}while(0)

#define Printf(fmt,...) do{\
	static char buff[1024] = "";\
	sprintf(buff,fmt,__VA_ARGS__);\
	HAL_UART_Transmit(&huart1,(unsigned char*)buff,strlen(buff),100);\
}while(0)

//GPIO
#include "gpio.h"
//LED
inline void SetLeds(unsigned char led){
	GPIOC->ODR = ~led << 8;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

//KEYBOARD
//Should Call In A Loop
#define GetKeyState(s,t) do{\
	static unsigned char keyState[4] = {0};\
	static unsigned char keyStateLast[4] = {0};\
	keyState[0] = (keyState[0]<<1) | ( HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)? 0:1 );\
	keyState[1] = (keyState[1]<<1) | ( HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)? 0:1 );\
	keyState[2] = (keyState[2]<<1) | ( HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)? 0:1 );\
	keyState[3] = (keyState[3]<<1) | ( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)? 0:1 );\
	s = (keyState[0]==0xFF?1:0) |  (keyState[1]==0xFF?2:0) |  (keyState[2]==0xFF?4:0)  |  (keyState[3]==0xFF?8:0) ; \
	t = 0;\
	if( keyStateLast[0] == 0 && keyState[0] == 0xFF ){ t |= 1; }\
	if( keyStateLast[1] == 0 && keyState[1] == 0xFF ){ t |= 2; }\
	if( keyStateLast[2] == 0 && keyState[2] == 0xFF ){ t |= 4; }\
	if( keyStateLast[3] == 0 && keyState[3] == 0xFF ){ t |= 8; }\
	if( keyState[0] == 0 || keyState[0] == 0xFF ){ keyStateLast[0] =  keyState[0]; }\
	if( keyState[1] == 0 || keyState[1] == 0xFF ){ keyStateLast[1] =  keyState[1]; }\
	if( keyState[2] == 0 || keyState[2] == 0xFF ){ keyStateLast[2] =  keyState[2]; }\
	if( keyState[3] == 0 || keyState[3] == 0xFF ){ keyStateLast[3] =  keyState[3]; }\
}while(0)

////////////////////
////LCD
extern unsigned short ASCII_Table[];
#define LCD_W (240)
#define LCD_H (320)
#define LCD_Write(rs,val) do{\
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);\
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);\
	GPIOC->ODR = val;\
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,rs?GPIO_PIN_SET:GPIO_PIN_RESET);\
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);\
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);\
}while(0)

#define LCD_WriteReg(reg,val)   LCD_Write(0,reg); LCD_Write(1,val)
#define LCD_SetCursor(xp,yp)    LCD_WriteReg(0x20,xp); LCD_WriteReg(0x21,yp)
#define LCD_WriteRAM_Prepare()  LCD_Write(0,0x22)
#define LCD_WriteRAM(rgb565)    LCD_Write(1,rgb565)
#define LCD_Delay               HAL_Delay

#define MAKE_RGB565(r,g,b) ( ((r&0xF8)<<8) | ((g&0xFC)<<3) | ((b&0xF8)>>3) )

#define LCD_Init(m) do{\
	LCD_WriteReg(0x00,0x0001);\
	LCD_WriteReg(0x03,m?0x1008:0x1030);\
	LCD_WriteReg(0x07,0x0173);\
	LCD_WriteReg(0x10,0x1090);\
	LCD_WriteReg(0x11,0x0227);\
	LCD_WriteReg(0x12,0x001d);\
	LCD_WriteReg(0x60,0x2700);\
	LCD_WriteReg(0x61,0x0001);\
	LCD_Delay(200);\
}while(0)

#define LCD_Clear(clr) do{\
	LCD_SetCursor(0,0);\
	LCD_WriteRAM_Prepare();\
	int i=LCD_W*LCD_H;\
	while(--i){ LCD_WriteRAM(clr); }\
}while(0)


#define LCD_DrawChar(mode,xp,yp,ch,clrf,clrb) do{\
	unsigned short* pat = ASCII_Table+24*(ch-32);\
	unsigned short nx = xp, ny = yp;\
	if(mode){ nx = yp; ny = xp; }\
	for(unsigned char j=0; j<24; ++j ){\
		if(mode){ LCD_SetCursor(nx+j,LCD_H-ny); }\
		else{ LCD_SetCursor(xp,yp+j); }\
		LCD_WriteRAM_Prepare();\
		unsigned short dat = pat[j];\
		for(unsigned char i=0; i<16; ++i ){\
			if( (dat & (1<<i)) == 0x00 ){ LCD_WriteRAM(clrb); }\
			else{ LCD_WriteRAM(clrf); }\
		}\
	}\
}while(0)

#define LCD_DrawString(mode,xp,yp,str,clrf,clrb) do{\
	char* p = str;\
	char  c = *p;\
	unsigned short dp = 0;\
	while(c){\
		LCD_DrawChar(mode,xp+dp,yp,c,clrf,clrb);\
		dp+=16;\
		++p; c = *p;\
	}\
}while(0)

#define LCDPrintf(mode,xp,yp,clrf,clrb,fmt,...) do{\
	static char buff[256] = "";\
	sprintf(buff,fmt,__VA_ARGS__);\
	LCD_DrawString(mode,xp,yp,buff,clrf,clrb);\
}while(0)

//I2C
#define SDA_Input_Mode() do{\
	GPIO_InitTypeDef inst = {0};\
	inst.Pin  = GPIO_PIN_7;\
	inst.Mode = GPIO_MODE_INPUT;\
	inst.Pull = GPIO_NOPULL;\
	HAL_GPIO_Init(GPIOB, &inst);\
}while(0)

#define SDA_Output_Mode() do{\
	GPIO_InitTypeDef inst = {0};\
	inst.Pin   = GPIO_PIN_7;\
	inst.Mode  = GPIO_MODE_OUTPUT_OD;\
	inst.Pull  = GPIO_NOPULL;\
	inst.Speed = GPIO_SPEED_FREQ_LOW;\
	HAL_GPIO_Init(GPIOB, &inst);\
}while(0)

#define SDA_Input() HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)
#define SDA_Output(v) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,v?GPIO_PIN_SET:GPIO_PIN_RESET)
#define SCL_Output(v) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,v?GPIO_PIN_SET:GPIO_PIN_RESET)

#define I2C_Delay(n) do{\
	unsigned int cnt = n; while(cnt--);\
}while(0)

#define I2C_Start() do{\
	SDA_Output(1); I2C_Delay(500);\
	SCL_Output(1); I2C_Delay(500);\
	SDA_Output(0); I2C_Delay(500);\
	SCL_Output(0); I2C_Delay(500);\
}while(0)

#define I2C_Stop() do{\
	SCL_Output(0); I2C_Delay(500);\
	SDA_Output(0); I2C_Delay(500);\
	SCL_Output(1); I2C_Delay(500);\
	SDA_Output(1); I2C_Delay(500);\
}while(0)

#define I2C_WaitAck() do{\
	char err = 5;\
	SDA_Input_Mode(); I2C_Delay(500);\
	SCL_Output(1);    I2C_Delay(500);\
	while( SDA_Input() ){\
		err--;	I2C_Delay(500);\
		if( !err ){\
			SDA_Output_Mode();\
			I2C_Stop();\
		}\
	}\
	SDA_Output_Mode();\
	SCL_Output(0); I2C_Delay(500);\
}while(0)

#define I2C_SendAck() do{\
	SDA_Output(0); I2C_Delay(500);\
	SCL_Output(1); I2C_Delay(500);\
	SCL_Output(0); I2C_Delay(500);\
}while(0)

#define I2C_SendNotAck() do{\
	SDA_Output(1); I2C_Delay(500);\
	SCL_Output(1); I2C_Delay(500);\
	SCL_Output(0); I2C_Delay(500);\
}while(0)

#define I2C_SendByte(val) do{\
	unsigned char v = val;\
	unsigned char i = 8;\
	while(i--){\
		SCL_Output(0);      I2C_Delay(500);\
		SDA_Output(v&0x80); I2C_Delay(500);\
		v <<= 1;\
		SCL_Output(1);      I2C_Delay(500);\
	}\
	SCL_Output(0);          I2C_Delay(500);\
}while(0)

#define I2C_RecvByte(ret) do{\
	unsigned char i = 8;\
	ret = 0;\
	SDA_Input_Mode();\
	while(i--){\
		ret <<= 1;\
		SCL_Output(0);      I2C_Delay(500);\
		SCL_Output(1);      I2C_Delay(500);\
		ret |= SDA_Input();\
	}\
	SCL_Output(0);          I2C_Delay(500);\
	SDA_Output_Mode();\
}while(0)

//EEPROM 24C02
#define MemRead(addr,src,n) do{\
	unsigned char ad  = addr;\
	unsigned char *p  = (unsigned char*)src;\
	unsigned char sz  = n;\
	unsigned char ch  = 0;\
	I2C_Start();\
	I2C_SendByte(0xA0); I2C_WaitAck();\
	I2C_SendByte(ad);   I2C_WaitAck();\
	I2C_Start();\
	I2C_SendByte(0xA1); I2C_WaitAck();\
	while(sz--){\
		I2C_RecvByte(ch);\
		*p++ = ch;\
		if( sz ){ I2C_SendAck(); }\
		else{ I2C_SendNotAck(); }\
	}\
	I2C_Stop();  I2C_Delay(500);\
}while(0)

#define MemWrite(addr,src,n) do{\
	unsigned char ad  = addr;\
	unsigned char *p = (unsigned char*)src;\
	unsigned char sz = n;\
	I2C_Start();\
	I2C_SendByte(0xA0); I2C_WaitAck();\
	I2C_SendByte(ad);   I2C_WaitAck();\
	while(sz--){ I2C_SendByte(*p++); I2C_WaitAck(); }\
	I2C_Stop();	 I2C_Delay(500);\
}while(0)

//MCP
#define MCP_Write(v) do{\
	I2C_Start();\
	I2C_SendByte(0x5e);  I2C_WaitAck();\
	I2C_SendByte(v);  I2C_WaitAck();\
	I2C_Stop();\
}while(0)

//ADC
#define ADC1_Calibration() HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
#define ADC2_Calibration() HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

#define ADC1_Read(v1,v2) do{\
	HAL_ADC_Start(&hadc1);\
	if( !HAL_ADC_PollForConversion(&hadc1,10) ){ v1 = HAL_ADC_GetValue(&hadc1); }\
	if( !HAL_ADC_PollForConversion(&hadc1,10) ){ v2 = HAL_ADC_GetValue(&hadc1); }\
}while(0)

#define ADC2_Read(v1) do{\
	HAL_ADC_Start(&hadc2);\
	if( !HAL_ADC_PollForConversion(&hadc2,10) ){ v1 = HAL_ADC_GetValue(&hadc2); }\
}while(0)

//TIM
#include "tim.h"
inline void TIM1_PWM_Start(){ HAL_TIM_PWM_Start(&htim1,2); }
inline void TIM1_SetAutoLoad(unsigned short v){ TIM1->ARR = v; }
inline void TIM1_SetCompare1(unsigned short v){ TIM1->CCR1 = v; }

inline void TIM2_IC_Start(){
	HAL_TIM_IC_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2,TIM_CHANNEL_2);
}
inline void TIM2_GetCapture(unsigned short *p){
	p[0] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1)+1;
	p[1] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2)+1;
}

inline void TIM3_PWM_Start(){ HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1); }
inline void TIM3_SetAutoLoad(unsigned short v){ TIM3->ARR = v; }
inline void TIM3_SetCompare1(unsigned short v){ TIM3->CCR1 = v; }



#endif
