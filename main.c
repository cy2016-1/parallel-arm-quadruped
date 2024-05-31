
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/TIMER/gtim.h"

#include "./BSP/NRF24L01/nrf24L01.h"
  
#include "./BSP/KEY/key.h"
#include "demo.h"
//#include "./BSP/PCA9685/pca9685.h"
//#include "./BSP/IIC/myiic.h"

#include <math.h>

extern TIM_HandleTypeDef g_tim3_pwm_ch_handle;     /* 定时器3ch3句柄     PB0     */
extern TIM_HandleTypeDef g_tim2_pwm_ch_handle;     /* 定时器3ch3句柄     PB0     */
extern TIM_HandleTypeDef g_tim4_pwm_ch_handle;     /* 定时器3ch3句柄     PB0     */

extern float imu_buf[10];
extern float walk_xy[8],t_speed;
extern int Motion_state;
int16_t map(float val,int16_t map_I_min,int16_t map_I_max,int16_t map_O_min,int16_t map_O_max)
{
	int16_t map_val=(int16_t)(val);
  if(map_val<map_I_min)
    {
      return map_O_min;
    }
  if(map_val>map_I_max)
    {
      return map_O_max; 	
    }
//		return  (map_val*(map_O_max-map_O_min)/(map_I_max-map_I_min))+(map_O_max-map_O_min)/2;
  return  (map_val*10/9)+150;
}
float L1=30,L2=48,L3=62.0474,DIS_X=21/2;	
float P_x=0,P_y=60,P_z=60;					


float Xs=0,Xf=10,H=10,PI=3.14,delta_X=0;//Xf=-20

float x1,x2,x3,x4,y1,y2,y3,y4;
extern float pi;
float ABl_x,ABl_z,AB2_x,AB2_z,AB3_x,AB3_z,AB4_x,AB4_z;
float pwm_out[4][4]={{0,150,0,150},{0,150,0,150},{0,150,0,150},{0,150,0,150}};
void IK_calculate(float IK_x,float IK_y,int IK_i){
	float IK_angle1,IK_angle2,IK_angle3,IK_angle4,IK_angle5,IK_x_1,IK_y_1,IK_angle6,IK_angle7,IK_angle8,IK_angle9=0.3336;
	if((IK_x-DIS_X)==0){
		IK_angle1=0;
	}else{
		IK_angle1=asin((IK_x-DIS_X)/(sqrt((IK_x-DIS_X)*(IK_x-DIS_X)+IK_y*IK_y)));
	}
	IK_angle2=acos(((IK_x-DIS_X)*(IK_x-DIS_X)+IK_y*IK_y+L1*L1-L3*L3)/(2*L1*sqrt((IK_x-DIS_X)*(IK_x-DIS_X)+IK_y*IK_y)));
	IK_angle3=1.57-IK_angle2-IK_angle1;
	pwm_out[IK_i][2]=-IK_angle3;
	pwm_out[IK_i][3]=map(-IK_angle3*57.3,-90,90,50,250);
	IK_angle4=acos((L1*L1+L3*L3-((IK_x-DIS_X)*(IK_x-DIS_X)+IK_y*IK_y))/(2*L1*L3));
	IK_angle5=IK_angle4-IK_angle3-IK_angle9;
	IK_x_1=L1*cos(IK_angle3)-L2*cos(IK_angle5)+DIS_X*2;
	IK_y_1=L1*sin(IK_angle3)+L2*sin(IK_angle5);
	
	if(IK_x_1==0){
		IK_angle6=0;
	}else{
		IK_angle6=asin(IK_x_1/(sqrt(IK_x_1*IK_x_1+IK_y_1*IK_y_1)));
	}
	IK_angle7=acos((IK_x_1*IK_x_1+IK_y_1*IK_y_1+L1*L1-L2*L2)/(2*L1*sqrt(IK_x_1*IK_x_1+IK_y_1*IK_y_1)));
	IK_angle8=1.57-IK_angle7+IK_angle6;
	pwm_out[IK_i][0]=IK_angle8;
	pwm_out[IK_i][1]=map(IK_angle8*57.3,-90,90,50,250);
}

void PWM_OUT(){
	__HAL_TIM_SET_COMPARE(&g_tim3_pwm_ch_handle, GTIM_TIM3_PWM_CH1, 300-pwm_out[0][1]);
	__HAL_TIM_SET_COMPARE(&g_tim3_pwm_ch_handle, GTIM_TIM3_PWM_CH2, 300-pwm_out[0][3]);

	__HAL_TIM_SET_COMPARE(&g_tim3_pwm_ch_handle, GTIM_TIM3_PWM_CH3, 300-pwm_out[1][1]);
	__HAL_TIM_SET_COMPARE(&g_tim3_pwm_ch_handle, GTIM_TIM3_PWM_CH4, 300-pwm_out[1][3]);
	
	__HAL_TIM_SET_COMPARE(&g_tim4_pwm_ch_handle, GTIM_TIM4_PWM_CH3, pwm_out[2][1]);
	__HAL_TIM_SET_COMPARE(&g_tim4_pwm_ch_handle, GTIM_TIM4_PWM_CH4, pwm_out[2][3]);
	
	__HAL_TIM_SET_COMPARE(&g_tim4_pwm_ch_handle, GTIM_TIM4_PWM_CH1, pwm_out[3][1]);
	__HAL_TIM_SET_COMPARE(&g_tim4_pwm_ch_handle, GTIM_TIM4_PWM_CH2, pwm_out[3][3]);
		
}


void cal_ges(float PIT,float ROL,float l,float b,float w){
	float YA=0,P,R,Y;
	
//	P=PIT;
//	R=ROL;
//	Y=YA;
	P=PIT*pi/180;
	R=ROL*pi/180;
	Y=YA*pi/180;

	ABl_x=l/2 - (l*cos(P)*cos(Y))/2 + (b*cos(P)*sin(Y))/2;
//	ABl_y=w/2 - (b*(cos(R)*cos(Y) + sin(P)*sin(R)*sin(Y)))/2 - (l*(cos(R)*sin(Y) - cos(Y)*sin(P)*sin(R)))/2;
	ABl_z= - (b*(cos(Y)*sin(R) - cos(R)*sin(P)*sin(Y)))/2 - (l*(sin(R)*sin(Y) + cos(R)*cos(Y)*sin(P)))/2;

	AB2_x=l/2 - (l*cos(P)*cos(Y))/2 - (b*cos(P)*sin(Y))/2;
//	AB2_y=(b*(cos(R)*cos(Y) + sin(P)*sin(R)*sin(Y)))/2 - w/2 - (l*(cos(R)*sin(Y) - cos(Y)*sin(P)*sin(R)))/2;
	AB2_z=(b*(cos(Y)*sin(R) - cos(R)*sin(P)*sin(Y)))/2 - (l*(sin(R)*sin(Y) + cos(R)*cos(Y)*sin(P)))/2;

	AB3_x=(l*cos(P)*cos(Y))/2 - l/2 + (b*cos(P)*sin(Y))/2;
//	AB3_y=w/2 - (b*(cos(R)*cos(Y) + sin(P)*sin(R)*sin(Y)))/2 + (l*(cos(R)*sin(Y) - cos(Y)*sin(P)*sin(R)))/2;
	AB3_z=(l*(sin(R)*sin(Y) + cos(R)*cos(Y)*sin(P)))/2 - (b*(cos(Y)*sin(R) - cos(R)*sin(P)*sin(Y)))/2;

	AB4_x=(l*cos(P)*cos(Y))/2 - l/2 - (b*cos(P)*sin(Y))/2;
//	AB4_y=(b*(cos(R)*cos(Y) + sin(P)*sin(R)*sin(Y)))/2 - w/2 + (l*(cos(R)*sin(Y) - cos(Y)*sin(P)*sin(R)))/2;
	AB4_z=(b*(cos(Y)*sin(R) - cos(R)*sin(P)*sin(Y)))/2 + (l*(sin(R)*sin(Y) + cos(R)*cos(Y)*sin(P)))/2;
}
int len,dir_state=0;
int change_state=1;

float trim_x[4]={0,0,0,0},raise_val=0,raise_val_max=35;
float trim_h[4]={12,3,4,2};
uint8_t main_tmp_buf[32],main_t;
uint16_t IS[4];
extern int turn;
extern int fb_state;
int main(void)
{
	
  HAL_Init();                             /* 初始化HAL库 */
  sys_stm32_clock_init(RCC_PLL_MUL9);     /* 设置时钟, 72Mhz */
  delay_init(72);                         /* 延时初始化 */
  usart_init(9600);                     /* 串口初始化为115200 */
 
  gtim_tim3_pwm_ch_init(2000 - 1, 720 - 1);/* 1Mhz的计数频率,2Khz的PWM. */
	gtim_tim4_pwm_ch_init(2000 - 1, 720 - 1);/* 1Mhz的计数频率,2Khz的PWM. */
  btim_timx_int_init(2000 - 1, 360 - 1);
  atim_timx_int_init(2000 - 1, 360 - 1);

	demo_run();  
			
//	PCA_Servo_Init(50,90);

  NRF24L01_SPI_Init();    		//初始化NRF24L01

  while(NRF24L01_Check())
    {
      printf("硬件查寻不到NRF24L01无线模块\n");

      HAL_Delay(1000);
    }
  printf("NRF24L01无线模块硬件连接正常\n");

  NRF24L01_RX_Mode();

  printf("进入数据接收模式\n");

  while (1)
    {
			if(NRF24L01_RxPacket(main_tmp_buf)==0)
				{
					for(main_t=0; main_t<4; main_t++)
						{
							IS[main_t] = (uint16_t) (main_tmp_buf[2*main_t] << 8) | main_tmp_buf[2*main_t+1];
						}
				}
			t_speed=(float)IS[0]/4096*0.09+0.01;
			if(t_speed>0.1){
					t_speed=0.1;
				}else if(t_speed<0.01){
					t_speed=0.01;
				}
			if(IS[3]<1800){
				turn=1;
			}else if(IS[3]>2400){
				turn=2;
			}else{
				turn=0;
			}
			if(IS[2]<1800){
				fb_state=2;
			}else if(IS[2]>2400) {
				fb_state=1;
			}else{
				fb_state=0;
			}
			delta_X=Xf-Xs;
			raise_val=(float)IS[1]/4096*2*raise_val_max-raise_val_max;
			if(main_tmp_buf[9]){
					Motion_state=1;
					delta_X=2*(Xf-Xs);
			}else{
				Motion_state=2;
				delta_X=Xf-Xs;
			}
			x1=delta_X*walk_xy[0]+Xs-10;
			y1=H*walk_xy[1];
			x2=delta_X*walk_xy[2]+Xs+10;
			y2=H*walk_xy[3];
			x3=delta_X*walk_xy[4]+Xs+10;
			y3=H*walk_xy[5];
			x4=delta_X*walk_xy[6]+Xs-10;
			y4=H*walk_xy[7];
			cal_ges(imu_buf[1]+raise_val,imu_buf[0],119,92,116.4);
			IK_calculate(x1+trim_x[0]+AB2_x,y1+trim_h[0]+P_y+AB2_z,0);  //FR
			IK_calculate(x2+trim_x[1]+AB4_x,y2+trim_h[1]+P_y+AB4_z,1);	//BR
			IK_calculate(x3+trim_x[2]+AB3_x,y3+trim_h[2]+P_y+AB3_z,2);	//BL
			IK_calculate(x4+trim_x[3]+ABl_x,y4+trim_h[3]+P_y+ABl_z,3);	//FR
			PWM_OUT();
			
    }
}


//			if (g_usart_rx_sta & 0x8000)        /* 接收到了数据? */
//			{
//					int i=1,point_state=0,point_index=1;
//					float M_rx_val=0;
//					char get_t,M_get_index;
//					len = g_usart_rx_sta & 0x3fff;  /* 得到此次接收到的数据长度 */
//					get_t=(char)g_usart_rx_buf[0];
//					M_get_index=get_t;
//					for(i=1;i<len;i++){
//							get_t=(char)g_usart_rx_buf[i];
//							if(get_t=='.'){
//									point_state=1;
//									point_index=1;
//							}
//							if(point_state&&get_t>=48&&get_t<=57){
//									point_index*=10;
//									M_rx_val+=(float)(get_t-'0')/point_index;
//									if(point_index==100){
//											break;
//									}
//							}else if(get_t>=48&&get_t<=57){
//									M_rx_val=M_rx_val*10+(get_t-'0');
//							}
//					}
//					switch(M_get_index){
//						case 'Q':
//									if((char)g_usart_rx_buf[1]=='-'){
//											dir_state=1;
//											Xf=0-M_rx_val;
//											}else{
//													dir_state=0;
//													Xf=M_rx_val;
//									}
//									break;
//						case 'W':
//									if((char)g_usart_rx_buf[1]=='-'){
//											dir_state=1;
//											H=0-M_rx_val;
//											}else{
//													dir_state=0;
//													H=M_rx_val;
//									}
//									break;
//								case 'v':
//									if((char)g_usart_rx_buf[1]=='-'){
//											dir_state=1;
//											trim_h[0]=0-M_rx_val;
//											}else{
//													dir_state=0;
//													trim_h[0]=M_rx_val;
//									}
//									break;
//								case 'b':
//									if((char)g_usart_rx_buf[1]=='-'){
//											dir_state=1;
//											trim_h[1]=0-M_rx_val;
//											}else{
//													dir_state=0;
//													trim_h[1]=M_rx_val;
//									}
//									break;
//								case 'n':
//									if((char)g_usart_rx_buf[1]=='-'){
//											dir_state=1;
//											trim_h[2]=0-M_rx_val;
//											}else{
//													dir_state=0;
//													trim_h[2]=M_rx_val;
//									}
//									break;
//								case 'm':
//									if((char)g_usart_rx_buf[1]=='-'){
//											dir_state=1;
//											trim_h[3]=0-M_rx_val;
//											}else{
//													dir_state=0;
//													trim_h[3]=M_rx_val;
//									}
//									break;
//							case 'P':
//									if(M_rx_val==1){
//										Motion_state=1;fb_state=2;turn=0;
//									}else if(M_rx_val==2){
//										Motion_state=1;fb_state=0;turn=0;
//									}
//									break;
//							case 'O':
//									if((char)g_usart_rx_buf[1]=='-'){
//											dir_state=1;
//											change_state=0-M_rx_val;
//											}else{
//													dir_state=0;
//													change_state=M_rx_val;
//									}
//									break;
//					}
//					g_usart_rx_sta = 0;
//					i=1;point_state=0;point_index=1;M_get_index=' ';M_rx_val=0;
//			}


