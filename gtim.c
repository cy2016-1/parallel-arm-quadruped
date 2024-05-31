
#include "./BSP/TIMER/gtim.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/NRF24L01/nrf24L01.h"
#include <math.h>
#include "demo.h"

#define POLEPAIRS 14
#define hall_Dir_0 4
#define hall_Dir_1 5
#define SPEEDMULT 600000

#define ADC_DMA_BUF_SIZE        50 * 2      /* ADC DMA�ɼ� BUF��С, Ӧ����ADCͨ������������ */
uint16_t g_adc_dma_buf[ADC_DMA_BUF_SIZE];   /* ADC DMA BUF */

extern uint8_t g_adc_dma_sta;               /* DMA����״̬��־, 0,δ���; 1, ����� */

/*********************************������ͨ�ö�ʱ��PWM���ʵ�����*************************************/

TIM_HandleTypeDef g_tim3_pwm_ch_handle;     /* ��ʱ��x��� */

TIM_HandleTypeDef g_tim4_pwm_ch_handle;     /* ��ʱ��x��� */

TIM_HandleTypeDef a_timx_handle; /* ��ʱ��x��� */

TIM_HandleTypeDef b_timx_handle; /* ��ʱ��x��� */


/**
 * @brief       ͨ�ö�ʱ��TIMX ͨ��Y PWM��� ��ʼ��������ʹ��PWMģʽ1��
 * @note
 *              ͨ�ö�ʱ����ʱ������APB1,��PPRE1 �� 2��Ƶ��ʱ��
 *              ͨ�ö�ʱ����ʱ��ΪAPB1ʱ�ӵ�2��, ��APB1Ϊ36M, ���Զ�ʱ��ʱ�� = 72Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void gtim_tim3_pwm_ch_init(uint16_t arr, uint16_t psc)
{
  TIM_OC_InitTypeDef tim3_oc_pwm_chy  = {0};                          /* ��ʱ��PWM������� */

  g_tim3_pwm_ch_handle.Instance = GTIM_TIM3_PWM;                     /* ��ʱ��x */
  g_tim3_pwm_ch_handle.Init.Prescaler = psc;                         /* ��ʱ����Ƶ */
  g_tim3_pwm_ch_handle.Init.CounterMode = TIM_COUNTERMODE_UP;        /* ��������ģʽ */
  g_tim3_pwm_ch_handle.Init.Period = arr;                            /* �Զ���װ��ֵ */
  HAL_TIM_PWM_Init(&g_tim3_pwm_ch_handle);                           /* ��ʼ��PWM */

  tim3_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;                           /* ģʽѡ��PWM1 */
  tim3_oc_pwm_chy.Pulse = arr / 2;                                    /* ���ñȽ�ֵ,��ֵ����ȷ��ռ�ձ� */
  /* Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50% */
  tim3_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_HIGH;                    /* ����Ƚϼ���Ϊ�� */

  HAL_TIM_PWM_ConfigChannel(&g_tim3_pwm_ch_handle, &tim3_oc_pwm_chy, GTIM_TIM3_PWM_CH1); /* ����TIMxͨ��y */
  HAL_TIM_PWM_Start(&g_tim3_pwm_ch_handle, GTIM_TIM3_PWM_CH1);       /* ������ӦPWMͨ�� */

  HAL_TIM_PWM_ConfigChannel(&g_tim3_pwm_ch_handle, &tim3_oc_pwm_chy, GTIM_TIM3_PWM_CH2); /* ����TIMxͨ��y */
  HAL_TIM_PWM_Start(&g_tim3_pwm_ch_handle, GTIM_TIM3_PWM_CH2);       /* ������ӦPWMͨ�� */
	
  HAL_TIM_PWM_ConfigChannel(&g_tim3_pwm_ch_handle, &tim3_oc_pwm_chy, GTIM_TIM3_PWM_CH3); /* ����TIMxͨ��y */
  HAL_TIM_PWM_Start(&g_tim3_pwm_ch_handle, GTIM_TIM3_PWM_CH3);       /* ������ӦPWMͨ�� */

  HAL_TIM_PWM_ConfigChannel(&g_tim3_pwm_ch_handle, &tim3_oc_pwm_chy, GTIM_TIM3_PWM_CH4); /* ����TIMxͨ��y */
  HAL_TIM_PWM_Start(&g_tim3_pwm_ch_handle, GTIM_TIM3_PWM_CH4);       /* ������ӦPWMͨ�� */
}


void gtim_tim4_pwm_ch_init(uint16_t arr, uint16_t psc)
{
  TIM_OC_InitTypeDef tim4_oc_pwm_chy  = {0};                          /* ��ʱ��PWM������� */

  g_tim4_pwm_ch_handle.Instance = GTIM_TIM4_PWM;                     /* ��ʱ��x */
  g_tim4_pwm_ch_handle.Init.Prescaler = psc;                         /* ��ʱ����Ƶ */
  g_tim4_pwm_ch_handle.Init.CounterMode = TIM_COUNTERMODE_UP;        /* ��������ģʽ */
  g_tim4_pwm_ch_handle.Init.Period = arr;                            /* �Զ���װ��ֵ */
  HAL_TIM_PWM_Init(&g_tim4_pwm_ch_handle);                           /* ��ʼ��PWM */

  tim4_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;                           /* ģʽѡ��PWM1 */
  tim4_oc_pwm_chy.Pulse = arr / 2;                                    /* ���ñȽ�ֵ,��ֵ����ȷ��ռ�ձ� */
  /* Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50% */
  tim4_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_HIGH;                    /* ����Ƚϼ���Ϊ�� */

  HAL_TIM_PWM_ConfigChannel(&g_tim4_pwm_ch_handle, &tim4_oc_pwm_chy, GTIM_TIM4_PWM_CH1); /* ����TIMxͨ��y */
  HAL_TIM_PWM_Start(&g_tim4_pwm_ch_handle, GTIM_TIM4_PWM_CH1);       /* ������ӦPWMͨ�� */

  HAL_TIM_PWM_ConfigChannel(&g_tim4_pwm_ch_handle, &tim4_oc_pwm_chy, GTIM_TIM4_PWM_CH2); /* ����TIMxͨ��y */
  HAL_TIM_PWM_Start(&g_tim4_pwm_ch_handle, GTIM_TIM4_PWM_CH2);       /* ������ӦPWMͨ�� */
	
  HAL_TIM_PWM_ConfigChannel(&g_tim4_pwm_ch_handle, &tim4_oc_pwm_chy, GTIM_TIM4_PWM_CH3); /* ����TIMxͨ��y */
  HAL_TIM_PWM_Start(&g_tim4_pwm_ch_handle, GTIM_TIM4_PWM_CH3);       /* ������ӦPWMͨ�� */

  HAL_TIM_PWM_ConfigChannel(&g_tim4_pwm_ch_handle, &tim4_oc_pwm_chy, GTIM_TIM4_PWM_CH4); /* ����TIMxͨ��y */
  HAL_TIM_PWM_Start(&g_tim4_pwm_ch_handle, GTIM_TIM4_PWM_CH4);       /* ������ӦPWMͨ�� */
}
/**
 * @brief       ��ʱ���ײ�������ʱ��ʹ�ܣ���������
                �˺����ᱻHAL_TIM_PWM_Init()����
 * @param       htim:��ʱ�����
 * @retval      ��
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == GTIM_TIM3_PWM)
    {
      GPIO_InitTypeDef gpio_init_struct;
      GTIM_TIM3_PWM_CH_GPIO_CLK_ENABLE();               /* ����ͨ��y��CPIOʱ�� */
      GTIM_TIM3_PWM_CH_CLK_ENABLE();
//        GTIM_TIM3_PWM_CH1_GPIO_REMAP();

      gpio_init_struct.Pin = GTIM_TIM3_PWM_CH1_GPIO_PIN; /* ͨ��y��CPIO�� */
      gpio_init_struct.Mode = GPIO_MODE_AF_PP;           /* ����������� */
      gpio_init_struct.Pull = GPIO_PULLDOWN;               /* ���� */
      gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;     /* ���� */
      HAL_GPIO_Init(GTIM_TIM3_PWM_CH1_GPIO_PORT, &gpio_init_struct);
			
			gpio_init_struct.Pin = GTIM_TIM3_PWM_CH2_GPIO_PIN; /* ͨ��y��CPIO�� */
      gpio_init_struct.Mode = GPIO_MODE_AF_PP;           /* ����������� */
      gpio_init_struct.Pull = GPIO_PULLDOWN;               /* ���� */
      gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;     /* ���� */
      HAL_GPIO_Init(GTIM_TIM3_PWM_CH2_GPIO_PORT, &gpio_init_struct);

      gpio_init_struct.Pin = GTIM_TIM3_PWM_CH3_GPIO_PIN; /* ͨ��y��CPIO�� */
      gpio_init_struct.Mode = GPIO_MODE_AF_PP;           /* ����������� */
      gpio_init_struct.Pull = GPIO_PULLDOWN;               /* ���� */
      gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;     /* ���� */
      HAL_GPIO_Init(GTIM_TIM3_PWM_CH3_GPIO_PORT, &gpio_init_struct);

      gpio_init_struct.Pin = GTIM_TIM3_PWM_CH4_GPIO_PIN; /* ͨ��y��CPIO�� */
      gpio_init_struct.Mode = GPIO_MODE_AF_PP;           /* ����������� */
      gpio_init_struct.Pull = GPIO_PULLDOWN;               /* ���� */
      gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;     /* ���� */
      HAL_GPIO_Init(GTIM_TIM3_PWM_CH4_GPIO_PORT, &gpio_init_struct);


//        GTIM_TIM3_PWM_CH4_GPIO_REMAP();                    /* IO��REMAP����, �Ƿ��Ҫ�鿴ͷ�ļ����õ�˵�� */
//        GTIM_TIM3_PWM_CH3_GPIO_REMAP();
    }
  if (htim->Instance == GTIM_TIM4_PWM)
    {
      GPIO_InitTypeDef gpio_init_struct;
      GTIM_TIM4_PWM_CH_GPIO_CLK_ENABLE();               /* ����ͨ��y��CPIOʱ�� */
      GTIM_TIM4_PWM_CH_CLK_ENABLE();

      gpio_init_struct.Pin = GTIM_TIM4_PWM_CH1_GPIO_PIN; /* ͨ��y��CPIO�� */
      gpio_init_struct.Mode = GPIO_MODE_AF_PP;           /* ����������� */
      gpio_init_struct.Pull = GPIO_PULLDOWN;               /* ���� */
      gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;     /* ���� */
      HAL_GPIO_Init(GTIM_TIM4_PWM_CH1_GPIO_PORT, &gpio_init_struct);

      gpio_init_struct.Pin = GTIM_TIM4_PWM_CH2_GPIO_PIN; /* ͨ��y��CPIO�� */
      gpio_init_struct.Mode = GPIO_MODE_AF_PP;           /* ����������� */
      gpio_init_struct.Pull = GPIO_PULLDOWN;               /* ���� */
      gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;     /* ���� */
      HAL_GPIO_Init(GTIM_TIM4_PWM_CH2_GPIO_PORT, &gpio_init_struct);
			
			gpio_init_struct.Pin = GTIM_TIM4_PWM_CH3_GPIO_PIN; /* ͨ��y��CPIO�� */
      gpio_init_struct.Mode = GPIO_MODE_AF_PP;           /* ����������� */
      gpio_init_struct.Pull = GPIO_PULLDOWN;               /* ���� */
      gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;     /* ���� */
      HAL_GPIO_Init(GTIM_TIM4_PWM_CH3_GPIO_PORT, &gpio_init_struct);
			
      gpio_init_struct.Pin = GTIM_TIM4_PWM_CH4_GPIO_PIN; /* ͨ��y��CPIO�� */
      gpio_init_struct.Mode = GPIO_MODE_AF_PP;           /* ����������� */
      gpio_init_struct.Pull = GPIO_PULLDOWN;               /* ���� */
      gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;     /* ���� */
      HAL_GPIO_Init(GTIM_TIM4_PWM_CH4_GPIO_PORT, &gpio_init_struct);
//        GTIM_TIM2_PWM_CH2_GPIO_REMAP();                    /* IO��REMAP����, �Ƿ��Ҫ�鿴ͷ�ļ����õ�˵�� */
    }

}

void atim_timx_int_init(uint16_t arr, uint16_t psc)
{
  a_timx_handle.Instance = ATIM_TIMX_INT;                      /* ͨ�ö�ʱ��X */
  a_timx_handle.Init.Prescaler = psc;                          /* ����Ԥ��Ƶϵ�� */
  a_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;         /* ��������ģʽ */
  a_timx_handle.Init.Period = arr;                             /* �Զ�װ��ֵ */
  HAL_TIM_Base_Init(&a_timx_handle);

  HAL_TIM_Base_Start_IT(&a_timx_handle);    /* ʹ�ܶ�ʱ��x��������ж� */
}

void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    b_timx_handle.Instance = BTIM_TIMX_INT;                      /* ͨ�ö�ʱ��X */
    b_timx_handle.Init.Prescaler = psc;                          /* ����Ԥ��Ƶϵ�� */
    b_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;         /* ��������ģʽ */
    b_timx_handle.Init.Period = arr;                             /* �Զ�װ��ֵ */
    HAL_TIM_Base_Init(&b_timx_handle);

    HAL_TIM_Base_Start_IT(&b_timx_handle);    /* ʹ�ܶ�ʱ��x��������ж� */
}

/**
 * @brief       ��ʱ��TIMX�жϷ�����
 * @param       ��
 * @retval      ��
 */
void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&b_timx_handle); /* ��ʱ���жϹ��������� */
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == ATIM_TIMX_INT)
    {
      ATIM_TIMX_INT_CLK_ENABLE();                     /* ʹ��TIMʱ�� */
      HAL_NVIC_SetPriority(ATIM_TIMX_INT_IRQn, 1, 3); /* ��ռ1�������ȼ�3����2 */
      HAL_NVIC_EnableIRQ(ATIM_TIMX_INT_IRQn);         /* ����ITM3�ж� */
    }
	if (htim->Instance == BTIM_TIMX_INT)
    {
        BTIM_TIMX_INT_CLK_ENABLE();                     /* ʹ��TIMʱ�� */
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 1, 3); /* ��ռ1�������ȼ�3����2 */
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);         /* ����ITM3�ж� */
    }
}
void ATIM_TIMX_INT_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&a_timx_handle); /* ��ʱ���жϹ��������� */
}
float xs=0,xf=1,h=1;
float Xep=0,Zep=0;
float sigma=0,Ts=1,faai=1,pi=3.141592,t=0,t_T=0,t_speed=0.02;
float walk_xy[8],p_x_f=-10,p_x_b=10,p_x_re;
int Motion_state=0,turn=0,fb_state=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_INT)
    {
			get_imu_data();
    }
		if(htim->Instance == BTIM_TIMX_INT){
			if(Motion_state==1&&turn==0&&fb_state==1){
				if(t<=Ts*faai){     //trot behind
					t_T=0;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					p_x_re=0;
					walk_xy[0]+=-t_speed;
					walk_xy[1]=0;
					walk_xy[2]=Xep-xf;
					walk_xy[3]=-Zep;
					walk_xy[4]+=-t_speed;
					walk_xy[5]=0;
					walk_xy[6]=Xep-xf;
					walk_xy[7]=-Zep;
					t+=t_speed;
				}else if(t>Ts*faai&&t<=2*Ts*faai){
					t_T=Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					p_x_re=0;
					walk_xy[0]=Xep-xf;
					walk_xy[1]=-Zep;
					walk_xy[2]+=-t_speed;
					walk_xy[3]=0;
					walk_xy[4]=Xep-xf;
					walk_xy[5]=-Zep;
					walk_xy[6]+=-t_speed;
					walk_xy[7]=0;
					t+=t_speed;
				}else if(t>2*Ts*faai){
					t=0;
				}
			}else if(Motion_state==1&&turn==0&&fb_state==2){
				if(t<=Ts*faai){     //trot front
					t_T=0;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					p_x_re=0;
					walk_xy[0]+=t_speed;
					walk_xy[1]=0;
					walk_xy[2]=-Xep+xf;
					walk_xy[3]=-Zep;
					walk_xy[4]+=t_speed;
					walk_xy[5]=0;
					walk_xy[6]=-Xep+xf;
					walk_xy[7]=-Zep;
					t+=t_speed;
				}else if(t>Ts*faai&&t<=2*Ts*faai){
					t_T=Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					p_x_re=0;
					walk_xy[0]=-Xep+xf;
					walk_xy[1]=-Zep;
					walk_xy[2]+=t_speed;
					walk_xy[3]=0;
					walk_xy[4]=-Xep+xf;
					walk_xy[5]=-Zep;
					walk_xy[6]+=t_speed;
					walk_xy[7]=0;
					t+=t_speed;
				}else if(t>2*Ts*faai){
					t=0;
				}
			}else if(Motion_state==1&&fb_state==0&&turn==1){
				if(t<=Ts*faai){     //trot-left
					t_T=0;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					p_x_re=0;
					walk_xy[0]+=-t_speed;
					walk_xy[1]=0;
					walk_xy[2]=Xep-xf;
					walk_xy[3]=-Zep;
					walk_xy[4]+=t_speed;
					walk_xy[5]=0;
					walk_xy[6]=-Xep+xf;
					walk_xy[7]=-Zep;
					t+=t_speed;
				}else if(t>Ts*faai&&t<=2*Ts*faai){
					t_T=Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					p_x_re=0;
					walk_xy[0]=Xep-xf;
					walk_xy[1]=-Zep;
					walk_xy[2]+=-t_speed;
					walk_xy[3]=0;
					walk_xy[4]=-Xep+xf;
					walk_xy[5]=-Zep;
					walk_xy[6]+=t_speed;
					walk_xy[7]=0;
					t+=t_speed;
				}else if(t>2*Ts*faai){
					t=0;
				}
			}else if(Motion_state==1&&fb_state==0&&turn==2){
				if(t<=Ts*faai){     //trot right
					t_T=0;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					p_x_re=0;
					walk_xy[0]+=t_speed;
					walk_xy[1]=0;
					walk_xy[2]=-Xep+xf;
					walk_xy[3]=-Zep;
					walk_xy[4]+=-t_speed;
					walk_xy[5]=0;
					walk_xy[6]=Xep-xf;
					walk_xy[7]=-Zep;
					t+=t_speed;
				}else if(t>Ts*faai&&t<=2*Ts*faai){
					t_T=Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					p_x_re=0;
					walk_xy[0]=-Xep+xf;
					walk_xy[1]=-Zep;
					walk_xy[2]+=t_speed;
					walk_xy[3]=0;
					walk_xy[4]=Xep-xf;
					walk_xy[5]=-Zep;
					walk_xy[6]+=-t_speed;
					walk_xy[7]=0;
					t+=t_speed;
				}else if(t>2*Ts*faai){
					t=0;
				}
			}else if(Motion_state==2&&turn==0&&fb_state==1){
				if(t<=Ts*faai){							//walk-front
					t_T=0;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
		
					walk_xy[0]=-1.5*xf+3*Xep; //1
					walk_xy[1]=-1.2*Zep+h*0.2;
					walk_xy[2]+=-t_speed;
					walk_xy[3]=h*0.2;
					walk_xy[4]+=-t_speed;
					walk_xy[5]=-h*0.2;
					walk_xy[6]+=-t_speed;
					walk_xy[7]=h*0.2;
					t+=t_speed;
				}else if(t>Ts*faai&&t<=2*Ts*faai){
					t_T=Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;

					walk_xy[0]+=-t_speed;
					walk_xy[1]=-h*0.2;
					walk_xy[2]+=-t_speed;
					walk_xy[3]=h*0.2;
					walk_xy[4]=-1.5*xf+Xep*3;   		//2
					walk_xy[5]=-1.2*Zep+h*0.2;
					walk_xy[6]+=-t_speed;
					walk_xy[7]=h*0.2;
					t+=t_speed;
				}else if(t>2*Ts*faai&&t<=3*Ts*faai){
					t_T=2*Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;

					walk_xy[0]+=-t_speed;
					walk_xy[1]=h*0.2;
					walk_xy[2]+=-t_speed;
					walk_xy[3]=-h*0.2;
					walk_xy[4]+=-t_speed;
					walk_xy[5]=h*0.2;
					walk_xy[6]=-1.5*xf+Xep*3;					//3
					walk_xy[7]=-1.2*Zep+h*0.2;
					t+=t_speed;
				}else if(t>3*Ts*faai&&t<=4*Ts*faai){
					t_T=3*Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					
					walk_xy[0]+=-t_speed;
					walk_xy[1]=h*0.2;
					walk_xy[2]=-1.5*xf+Xep*3;				//4
					walk_xy[3]=-1.2*Zep+h*0.2;
					walk_xy[4]+=-t_speed;
					walk_xy[5]=h*0.2;
					walk_xy[6]+=-t_speed;
					walk_xy[7]=-h*0.2;
					t+=t_speed;
				}else if(t>4*Ts*faai){
					t=0;
				}

			}else if(Motion_state==2&&turn==0&&fb_state==2){
				if(t<=Ts*faai){							//walk-behind
					t_T=0;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]+=t_speed;
					walk_xy[1]=-h*0.2;
					walk_xy[2]+=t_speed;
					walk_xy[3]=h*0.2;
					walk_xy[4]=1.5*xf-3*Xep;   		//2
					walk_xy[5]=-1.2*Zep+h*0.2;
					walk_xy[6]+=t_speed;
					walk_xy[7]=h*0.2;
					t+=t_speed;
				}else if(t>Ts*faai&&t<=2*Ts*faai){
					t_T=Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]=1.5*xf-3*Xep; //1
					walk_xy[1]=-1.2*Zep+h*0.2;
					walk_xy[2]+=t_speed;
					walk_xy[3]=h*0.2;
					walk_xy[4]+=t_speed;
					walk_xy[5]=-h*0.2;
					walk_xy[6]+=t_speed;
					walk_xy[7]=h*0.2;
					t+=t_speed;
				}else if(t>2*Ts*faai&&t<=3*Ts*faai){
					t_T=2*Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]+=t_speed;
					walk_xy[1]=h*0.2;
					walk_xy[2]=1.5*xf-Xep*3;				//4
					walk_xy[3]=-1.2*Zep+h*0.2;
					walk_xy[4]+=t_speed;
					walk_xy[5]=h*0.2;
					walk_xy[6]+=t_speed;
					walk_xy[7]=-h*0.2;
					t+=t_speed;
				}else if(t>3*Ts*faai&&t<=4*Ts*faai){
					t_T=3*Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]+=t_speed;
					walk_xy[1]=h*0.2;
					walk_xy[2]+=t_speed;
					walk_xy[3]=-h*0.2;
					walk_xy[4]+=t_speed;
					walk_xy[5]=h*0.2;
					walk_xy[6]=1.5*xf-Xep*3;					//3
					walk_xy[7]=-1.2*Zep+h*0.2;
					t+=t_speed;
				}else if(t>4*Ts*faai){
					t=0;
				}

			}else if(Motion_state==2&&turn==1&&fb_state==0){
			  if(t<=Ts*faai){							//walk-left 
					t_T=0;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]=(-xf+2*Xep)/2; //1
					walk_xy[1]=-1.2*Zep+h*0.2;
					walk_xy[2]+=-t_speed/2;
					walk_xy[3]=h*0.2;
					walk_xy[4]=xf/2;
					walk_xy[5]=-h*0.2;
					walk_xy[6]+=t_speed/2;
					walk_xy[7]=h*0.2;
					t+=t_speed;
				}else if(t>Ts*faai&&t<=2*Ts*faai){
					t_T=Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]=xf/2;
					walk_xy[1]=-h*0.2;
					walk_xy[2]+=-t_speed/2;
					walk_xy[3]=h*0.2;
					walk_xy[4]=(xf-2*Xep)/2;   		//2
					walk_xy[5]=-1.2*Zep+h*0.2;
					walk_xy[6]+=t_speed/2;
					walk_xy[7]=h*0.2;
					t+=t_speed;
				}else if(t>2*Ts*faai&&t<=3*Ts*faai){
					t_T=2*Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]+=-t_speed/2;
					walk_xy[1]=h*0.2;
					walk_xy[2]=-xf/2;
					walk_xy[3]=-h*0.2;
					walk_xy[4]+=t_speed/2;
					walk_xy[5]=h*0.2;
					walk_xy[6]=(xf-Xep*2)/2;					//3
					walk_xy[7]=-1.2*Zep+h*0.2;
					t+=t_speed;
				}else if(t>3*Ts*faai&&t<=4*Ts*faai){
					t_T=3*Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]+=-t_speed/2;
					walk_xy[1]=h*0.2;
					walk_xy[2]=(-xf+Xep*2)/2;				//4
					walk_xy[3]=-1.2*Zep+h*0.2;
					walk_xy[4]+=t_speed/2;
					walk_xy[5]=h*0.2;
					walk_xy[6]=-xf/2;
					walk_xy[7]=-h*0.2;
					t+=t_speed;
				}else if(t>4*Ts*faai){
					t=0;
				}

			}else if(Motion_state==2&&turn==2&&fb_state==0){
			  if(t<=Ts*faai){							//walk-right 
					t_T=0;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]=xf-2*Xep; //1
					walk_xy[1]=-1.2*Zep+h*0.2;
					walk_xy[2]+=t_speed;
					walk_xy[3]=h*0.2;
					walk_xy[4]=-xf;
					walk_xy[5]=-h*0.2;
					walk_xy[6]+=-t_speed;
					walk_xy[7]=h*0.2;
					t+=t_speed;
				}else if(t>Ts*faai&&t<=2*Ts*faai){
					t_T=Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]=-xf;
					walk_xy[1]=-h*0.2;
					walk_xy[2]+=t_speed;
					walk_xy[3]=h*0.2;
					walk_xy[4]=-xf+2*Xep;   		//2
					walk_xy[5]=-1.2*Zep+h*0.2;
					walk_xy[6]+=-t_speed;
					walk_xy[7]=h*0.2;
					t+=t_speed;
				}else if(t>2*Ts*faai&&t<=3*Ts*faai){
					t_T=2*Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]+=t_speed;
					walk_xy[1]=h*0.2;
					walk_xy[2]=xf;
					walk_xy[3]=-h*0.2;
					walk_xy[4]+=-t_speed;
					walk_xy[5]=h*0.2;
					walk_xy[6]=-xf+Xep*2;					//3
					walk_xy[7]=-1.2*Zep+h*0.2;
					t+=t_speed;
				}else if(t>3*Ts*faai&&t<=4*Ts*faai){
					t_T=3*Ts*faai;
					sigma=2*pi*(t-t_T)/(faai*Ts);
					Xep=((xf-xs)*(sigma-(sin(sigma))))/(2*pi)+xs;
					Zep=h*(1-cos(sigma))/2;
					walk_xy[0]+=t_speed;
					walk_xy[1]=h*0.2;
					walk_xy[2]=xf-Xep*2;				//4
					walk_xy[3]=-1.2*Zep+h*0.2;
					walk_xy[4]+=-t_speed;
					walk_xy[5]=h*0.2;
					walk_xy[6]=xf;
					walk_xy[7]=-h*0.2;
					t+=t_speed;
				}else if(t>4*Ts*faai){
					t=0;
				}
			}else{
					walk_xy[0]=0; 
					walk_xy[1]=0;
					walk_xy[2]=0;
					walk_xy[3]=0;
					walk_xy[4]=0;
					walk_xy[5]=0;
					walk_xy[6]=0;
					walk_xy[7]=0;
			}
		}
}


