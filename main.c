#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "pwm.h"
#include "encoder.h"
#include "led.h"
#include "key.h"
#include "adc.h"
#include "motor.h"

#define speed 200
#define speed_max_mode 500
#define speed_error 100
#define speed_error_max_mode 250

u8 remote_control=0;
u8 key1,key2,wake;
u16 adcx[6];

	short speed_L=0,speed_R=0;

int main(void)
{
	u8 len,bluetooth_flag=0,dir_flag=0;


  HAL_Init();                   	//初始化HAL库    
  Stm32_Clock_Init(336,25,2,7);  	//设置时钟,168Mhz
	delay_init(168);               	//初始化延时函数
	uart1_init(115200);             	//初始化USART
	uart3_init(9600);
	TIM4_PWM_Init(1000-1,4-1);    	//84M/4=21M的计数频率，自动重装载为1000，那么PWM频率为21M/1000=21kHZ
	tim_encoder_init();//编码器初始化
	TIM10_Init(1000-1,1680-1);//定时器中断初始化
	led_init();
	key_init();
	adc_init();
	delay_ms(500);
    while(1)
    {
			if(USART_RX_STA&0x8000)
			{					   
				len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
				if(len==1)
				remote_control=USART3_RX_BUF[0];
				while(__HAL_UART_GET_FLAG(&UART3_Handler,UART_FLAG_TC)!=SET);		//等待发送结束
				USART_RX_STA=0;
				bluetooth_flag=0;
			}
			else bluetooth_flag++;
			if(bluetooth_flag>10) remote_control=0x00;
			if(remote_control==0x00) {set_motor_pwm(motor2,0);set_motor_pwm(motor1,0);speed_L=0;speed_R=0;dir_flag=0;}
			
			if(remote_control!=0x00)
			{
				if(!(remote_control&0x08))
				{					
					if(remote_control&0x80) {speed_L=speed;speed_R=speed;dir_flag=0;}
					else if(remote_control&0x40) {speed_L=-speed;speed_R=-speed;dir_flag=0;}
					else {speed_L=0;speed_R=0;dir_flag=0;}
					if(remote_control&0x20&&dir_flag!=1) {speed_L=speed_L-speed_error;speed_R=speed_R+speed_error;dir_flag=1;}
					if(remote_control&0x10&&dir_flag!=2) {speed_L=speed_L+speed_error;speed_R=speed_R-speed_error;dir_flag=2;}
					set_motor_speed(speed_L,speed_R);
				}
				if(remote_control&0x08){
					if(remote_control&0x80) {speed_L=speed_max_mode;speed_R=speed_max_mode;dir_flag=0;}
					else if(remote_control&0x40) {speed_L=-speed_max_mode;speed_R=-speed_max_mode;dir_flag=0;}
					else {speed_L=0;speed_R=0;dir_flag=0;}
					if(remote_control&0x20&&dir_flag!=1) {speed_L=speed_L-speed_error_max_mode;speed_R=speed_R+speed_error_max_mode;dir_flag=1;}
					if(remote_control&0x10&&dir_flag!=2) {speed_L=speed_L+speed_error_max_mode;speed_R=speed_R-speed_error_max_mode;dir_flag=2;}
					set_motor_speed(speed_L,speed_R);
				}
			}
			delay_ms(10);
    }
}
