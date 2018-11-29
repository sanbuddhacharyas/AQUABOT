#ifndef MPU
#define MPU
#include "stm32f1xx_hal.h"
#define MPU_ADD 0x68
#define MAG_ADD  0xC

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;



void MPU_Init(uint16_t slave_address);
float MPU_GET_VALUE(uint16_t slave_address );
float MPU_GYRO_CAL(void);
void MPU_SHOWDATA(float* angle_gyro);
//Magnetometer
void BYPASS_MPU(uint16_t );
void Init_Magnetometer(uint8_t);
void continuous_mode(uint8_t );
void Mag_get_value(uint8_t );
// PID(float P,float I , float D,float cal,TIM_HandleTypeDef* htim,int* left_motor, int* right_motor);

#endif

