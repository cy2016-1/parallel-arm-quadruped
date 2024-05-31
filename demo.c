
#include "demo.h"
#include "./BSP/ATK_MS6050/atk_ms6050.h"
#include "./BSP/ATK_MS6050/eMPL/inv_mpu.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/key/key.h"
//#include "./BSP/lcd/lcd.h"
#include "./BSP/led/led.h"

#include <math.h>
#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

float imu_buf[10];
float imu_buf2[40][2];

typedef struct
{
  float Kp;                       //????Proportional
  float Ki;                       //????Integral
  float Kd;                       //????Derivative

  float Ek;                       //????
  float Ek1;                      //????? e(k-1)
  float Ek2;                      //?????? e(k-2)
}PID_IncTypeDef;
PID_IncTypeDef PID;

float PID_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID)
{
  float PIDInc;                                  

  PID->Ek = SetValue - ActualValue;
  PIDInc = (PID->Kp * PID->Ek) - (PID->Ki * PID->Ek1) + (PID->Kd * PID->Ek2);
	
  PID->Ek2 = PID->Ek1;
  PID->Ek1 = PID->Ek;  
	return LIMIT(PIDInc,-10,10);
}
/**
 * @brief       通过串口1发送数据至匿名地面站V4
 * @param       fun: 功能字
 *              dat: 待发送的数据（最多28字节）
 *              len: dat数据的有效位数
 * @retval      无
 */
static void demo_usart1_niming_report(uint8_t fun, uint8_t *dat, uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;
    
    if (len > 28)
    {
        return;
    }
    
    send_buf[len+4] = 0;            /* 校验位清零 */
    send_buf[0] = 0xAA;             /* 帧头为0xAAAA */
    send_buf[1] = 0xAA;             /* 帧头为0xAAAA */
    send_buf[2] = fun;              /* 功能字 */
    send_buf[3] = len;              /* 数据长度 */
    for (i=0; i<len; i++)           /* 复制数据 */
    {
        send_buf[4 + i] = dat[i];
    }
    for (i=0; i<(len + 4); i++)     /* 计算校验和 */
    {
        send_buf[len + 4] += send_buf[i];
    }
    
    /* 发送数据 */
    HAL_UART_Transmit(&g_uart1_handle, send_buf, len + 5, HAL_MAX_DELAY);
}

/**
 * @brief       发送状态帧至匿名地面站V4
 * @param       rol     : 横滚角
 *              pit     : 俯仰角
 *              yaw     : 航向角
 *              alt     : 飞行高度，单位：cm
 *              fly_mode: 飞行模式
 *              armed   : 锁定状态，0xA0：加锁 0xA1：解锁
 * @retval      无
 */
static void demo_niming_report_status(int16_t rol, int16_t pit, int16_t yaw, uint32_t alt, uint8_t fly_mode, uint8_t armed)
{
    uint8_t send_buf[12];
    
    /* 横滚角 */
    send_buf[0] = (rol >> 8) & 0xFF;
    send_buf[1] = rol & 0xFF;
    /* 俯仰角 */
    send_buf[2] = (pit >> 8) & 0xFF;
    send_buf[3] = pit & 0xFF;
    /* 航向角 */
    send_buf[4] = (yaw >> 8) & 0xFF;
    send_buf[5] = yaw & 0xFF;
    /* 飞行高度 */
    send_buf[6] = (alt >> 24) & 0xFF;
    send_buf[7] = (alt >> 16) & 0xFF;
    send_buf[8] = (alt >> 8) & 0xFF;
    send_buf[9] = alt & 0xFF;
    /* 飞行模式 */
    send_buf[10] = fly_mode;
    /* 锁定状态 */
    send_buf[11] = armed;
    
    /* 状态帧的功能字为0x01 */
    demo_usart1_niming_report(0x01, send_buf, 12);
}

/**
 * @brief       发送传感器帧至匿名地面站V4
 * @param       acc_x : x轴上的加速度值
 *              acc_y : y轴上的加速度值
 *              acc_z : z轴上的加速度值
 *              gyro_x: x轴上的陀螺仪值
 *              gyro_y: y轴上的陀螺仪值
 *              gyro_z: z轴上的陀螺仪值
 *              mag_x : x轴上的磁力计值（ATK-MS6050不支持）
 *              mag_y : y轴上的磁力计值（ATK-MS6050不支持）
 *              mag_z : z轴上的磁力计值（ATK-MS6050不支持）
 * @retval      无
 */
static void demo_niming_report_senser(  int16_t  acc_x, int16_t  acc_y, int16_t  acc_z,
                                        int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                                        int16_t  mag_x, int16_t  mag_y, int16_t  mag_z)
{
    uint8_t send_buf[18];
    
    /* x轴上的加速度值 */
    send_buf[0] = (acc_x >> 8) & 0xFF;
    send_buf[1] = acc_x & 0xFF;
    /* y轴上的加速度值 */
    send_buf[2] = (acc_y >> 8) & 0xFF;
    send_buf[3] = acc_y & 0xFF;
    /* z轴上的加速度值 */
    send_buf[4] = (acc_z >> 8) & 0xFF;
    send_buf[5] = acc_z & 0xFF;
    /* x轴上的陀螺仪值 */
    send_buf[6] = (gyro_x >> 8) & 0xFF;
    send_buf[7] = gyro_x & 0xFF;
    /* y轴上的陀螺仪值 */
    send_buf[8] = (gyro_y >> 8) & 0xFF;
    send_buf[9] = gyro_y & 0xFF;
    /* z轴上的陀螺仪值 */
    send_buf[10] = (gyro_z >> 8) & 0xFF;
    send_buf[11] = gyro_z & 0xFF;
    /* x轴上的磁力计值 */
    send_buf[12] = (mag_x >> 8) & 0xFF;
    send_buf[13] = mag_x & 0xFF;
    /* y轴上的磁力计值 */
    send_buf[14] = (mag_y >> 8) & 0xFF;
    send_buf[15] = mag_y & 0xFF;
    /* z轴上的磁力计值 */
    send_buf[16] = (mag_z >> 8) & 0xFF;
    send_buf[17] = mag_z & 0xFF;
    
    /* 传感器的功能字为0x02 */
    demo_usart1_niming_report(0x02, send_buf, 18);
}



/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
uint8_t ret;
uint8_t key;
uint8_t niming_report = 0;
float pit, rol, yaw;
int16_t acc_x, acc_y, acc_z;
int16_t gyr_x, gyr_y, gyr_z;
int16_t temp;

typedef struct
{
	float E_mea;
	float X_k[40][2];
	float K_k[40][2];
	float E_est[40][2];
}Kalman_InitDef;

Kalman_InitDef kalman_Structure;

void Kaleman_Parameter_Init()
{
	kalman_Structure.X_k[0][0] = 90;kalman_Structure.X_k[0][1] = 90;
	kalman_Structure.E_est[0][0] = 1;kalman_Structure.E_est[0][1] = 1;
	kalman_Structure.E_mea = 0.1;
}

float Kalman_Gain_Calculation(float E_est, float E_mea)
{
	float result;
	result = E_est / (E_est + E_mea);
	return result;
}
void Kalman_X_K_Calculation(void)
{
	char i,j;
	float Kalman_Gain[2];
	for(j=0;j<2;j++){

		for (i = 1; i < 40; i++)
	{
		Kalman_Gain[j] = Kalman_Gain_Calculation(kalman_Structure.E_est[i-1][j], kalman_Structure.E_mea);
		kalman_Structure.X_k[i][j] = kalman_Structure.X_k[i-1][j] + Kalman_Gain[j] * ((float)imu_buf2[i][j] - kalman_Structure.X_k[i-1][j]);
		kalman_Structure.E_est[i][j] = ((float)1 - Kalman_Gain[j]) * kalman_Structure.E_est[i-1][j];

	}
	
	imu_buf[j]=kalman_Structure.X_k[39][j];

	kalman_Structure.X_k[0][j]=kalman_Structure.X_k[39][j];
	kalman_Structure.E_est[0][j]=1;
	
	}
}
void demo_run(void)
{
	PID.Kp=4;PID.Ki=0;PID.Kd=0;
    /* 初始化ATK-MS6050 */
    ret = atk_ms6050_init();
	Kaleman_Parameter_Init();
    if (ret != 0)
    {
        printf("ATK-MS6050 init failed!\r\n");
//        while (1)
//        {
//            LED0_TOGGLE();
//            delay_ms(200);
//        }
    }
    
    /* 初始化ATK-MS6050 DMP */
    ret = atk_ms6050_dmp_init();
    if (ret != 0)
    {
        printf("ATK-MS6050 DMP init failed!\r\n");
//        while (1)
//        {
//            LED0_TOGGLE();
//            delay_ms(200);
//        }
    }
    
}

int cnt_i=0;
void get_imu_data(){
	
		/* 获取ATK-MS6050 DMP处理后的数据 */
		ret  = atk_ms6050_dmp_get_data(&pit, &rol, &yaw);
		/* 获取ATK-MS6050加速度值 */
//		ret += atk_ms6050_get_accelerometer(&acc_x, &acc_y, &acc_z);
//		/* 获取ATK-MS6050陀螺仪值 */
//		ret += atk_ms6050_get_gyroscope(&gyr_x, &gyr_y, &gyr_z);
//		/* 获取ATK-MS6050温度值 */
//		ret += atk_ms6050_get_temperature(&temp);
//			printf(" int_i:%u",cnt_i);
	if (ret == 0&&cnt_i<40)
		{
			imu_buf2[cnt_i][0]=pit;
			imu_buf2[cnt_i][1]=rol;
			
			cnt_i++;
		}else if(cnt_i>=40){
			cnt_i=0;
			Kalman_X_K_Calculation();
		}
		

}


