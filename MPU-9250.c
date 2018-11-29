
#include "MPU-9250.h"
#include "stm32f1xx_hal.h"
/*
At first Initialize MPU-9250 with its address
To get value use function MPU_GET_VALUE
*/
volatile float gyro_x,gyro_y,gyro_z,accel_x=0,accel_y=0,accel_z=0,temp,mag_x, mag_y ,mag_z,angle =0;
long int gyro_cal_y;
float asax , asay ,asaz; //Initialing sensitivity adjustment values
float Xa,Ya,Za,t;
float Xg=0,Yg=0,Zg=0;
double tot;
float angle1;
float angle_gyro=0;
float Del=0;
;

int set_gyro_angle=0;
char string[40];

float ang=0;

void MPU_Init(uint16_t slave_address)
{
	char string[30] ;
	uint8_t buffer[2]={0x19,0x07};//Gyroscope output value is 8MHZ so sample rate is 1MHz
	
  while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 1st\n");
		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	
	buffer[0] = 0x6B;//Power Managment system
	buffer[1] = 0x00;//Internal clock of 8MHz oscillator
 while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 2nd");
		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	//Configure
	buffer[0] = 0x1A;
	buffer[1] = 0x00;//Digital low pass filter Gyro = 8kHz
 while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 3rd \n");
		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	}

	//Configure Gyro
	buffer[0] = 0x1B;
	buffer[1] = 0x00; // 
while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 4rd \n");
		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	//Configure Accelerometer 
	buffer[0] = 0x1C;
	buffer[1] = 0x00;	
	while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 5rd \n");
		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	//Interrupt Enable
	buffer[0] = 0x38;
	buffer[1] = 0x01;
while(HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,2,200) != HAL_OK)
	{
		sprintf(string,"I am in 6rd \n");
		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	
	
}

float MPU_GET_VALUE(uint16_t slave_address)
{
	uint8_t buffer=0x43;
	int8_t data[6];
	HAL_I2C_Master_Transmit(&hi2c1,slave_address<<1,(uint8_t *)&buffer,1,200);
	//HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1,slave_address<<1,(uint8_t *)&data,6,200);
//	accel_x = ~((((int)data[0]<<8|((int)data[1])))-1);//2`s complements value to signed value value
//	accel_y = ~((((int)data[2]<<8|((int)data[3])))-1);
//	accel_z =~((((int)data[4]<<8|((int)data[5])))-1);
//	temp = ((data[6]<<8)|(data[7]));
	gyro_x = ~((((int)data[0]<<8|((int)data[1])))-1);
	gyro_y = ~((((int)data[2]<<8|((int)data[3])))-1);
	gyro_z = ~((((int)data[4]<<8|((int)data[5])))-1);
	
	 Xg = gyro_x/131;
   Yg = gyro_y/131;
   Zg = gyro_z/131;
	
	return  Yg;
		

}

float MPU_GYRO_CAL(void)
{
	for(int i=0; i<200;i++)
	{
		gyro_y = MPU_GET_VALUE(0X68 );
		gyro_cal_y += gyro_y;
	
	}
	gyro_cal_y /=200;
	
	return gyro_cal_y;

	
}

void MPU_SHOWDATA(float* angle_gyro)
{
     
     MPU_GET_VALUE(0x68);
//     Xa = accel_x/16384;								/* Divide raw value by sensitivity scale factor to get real values */
//     Ya = accel_y/16384;
//     Za = accel_z/16384;
     
     Xg = gyro_x/131;
     Yg = gyro_y/131;
     Zg = gyro_z/131;
     
	  
//		sprintf(string,"%f \n",Xg);
//		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
//		sprintf(string,"%f \n",Yg);
//		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
//		sprintf(string,"Gyro_Z=%f ",Zg);
//		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
//		sprintf(string,"\n");
//		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
//		
	
//		 sprintf(string,"T= %d\n",TIM2->CNT);
// 		 HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
//		 
     
////     del_x = (double)(TIM2->CNT) / 10000 ;
////		 if (Yg<1 & Yg>-1) Yg =0;
////     (*angle_gyro) += Yg*del_x;	
////		 TIM2->CNT = 0;	
		 
		 
//		 sprintf(string,"angle_Yg = %f\t",Yg);
// 		 HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
//		 
//		 sprintf(string,"angle = %f\n",angle_gyro);
// 		 HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
		 
     //tot = sqrt(Xa*Xa+Ya*Ya+Za*Za);
     //angle1 = asin((float)Xa/tot)*57.2957795130;
	/*
     if(set_gyro_angle)
     {
	     angle_gyro = angle_gyro*0.96 +angle1*0.04;
     }
     else
     {
	     angle_gyro = angle1;
	     set_gyro_angle =1;
     }   
		 */

}
//For using Magnetometer I2C master interface must be disable and Bypass multiplexer must be enable

void BYPASS_MPU(uint16_t slave_address)
{
	uint8_t buffer[2];
	//Disable the I2C Master interface and enable Bypass 
	buffer[0] = 0x6A;
	buffer[1] = 0x00;
 while(HAL_I2C_Master_Transmit(&hi2c1, slave_address<<1 ,(uint8_t *)&buffer ,2,200) != HAL_OK)
	{
		sprintf(string,"Diable I2C Master interfaec \n");
		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	//enable Bypass
	buffer[0] = 0x37;
	buffer[1] = 0x02;
 while(HAL_I2C_Master_Transmit(&hi2c1, slave_address<<1 ,(uint8_t *)&buffer ,2,200) != HAL_OK)
	{
		sprintf(string,"Enable Bypass multiplexer \n");
		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	sprintf(string,"I have completed bypass code \n");
	HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	
}

//To get the data of sensitivity adjsutment value it must be in Fuse ROM access mode
void Init_Magnetometer(uint8_t MAG_AD)
{
	uint8_t buffer[2];
	uint8_t buff=0x10 ;//address of sensitivity adjustment value
	uint8_t data[3];
	buffer[0] = 0x0A ; 
	buffer[1] = 0x1F;//fuse mode access mode
	
 while(HAL_I2C_Master_Transmit(&hi2c1, MAG_AD << 1 ,(uint8_t *)&buffer ,2,200) != HAL_OK)
	{
		sprintf(string,"Diable I2C Master interfaec \n");
		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	HAL_I2C_Master_Transmit(&hi2c1,MAG_AD<<1,(uint8_t *)&buff,1,200);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1,MAG_AD<<1,(uint8_t *)&data,3,200);
	asax = (data[0]-128)*0.5/128+1;//Sensitivity adjustment value for x
	asay = (data[1]-128)*0.5/128+1;
	asaz = (data[2]-128)*0.5/128+1;
}
void continuous_mode(uint8_t MAG_AD)
{
	uint8_t buffer[2];
	buffer[0] = 0x0A ; 
	buffer[1] = 0x00;//power down mode
	
 while(HAL_I2C_Master_Transmit(&hi2c1, MAG_AD << 1 ,(uint8_t *)&buffer ,2,200) != HAL_OK)
	{
		sprintf(string,"Diable I2C Master interfaec \n");
		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	}
	
	//Start the Magnetometer to Continuous mode 2 (100HZ) and 16 bit output
	
	buffer[0] = 0x0A ; 
	buffer[1] = 0x16; //0001 0110 in binary  continous mode
	
 while(HAL_I2C_Master_Transmit(&hi2c1, MAG_AD << 1 ,(uint8_t *)&buffer ,2,200) != HAL_OK)
	{
		sprintf(string,"Continuous_mode \n");
		HAL_UART_Transmit(&huart3,(uint8_t *)&string,sizeof(string),0xFFFF);
	}

}
/*
void PID (float P , float I , float D,float cal ,TIM_HandleTypeDef* htim ,int* left_motor, int* right_motor)
{
	  Yg = MPU_GET_VALUE(MPU_ADD);
		Yg = Yg - cal;
		Del = (double)(htim->Instance->CNT) / 64000 ;
		if (Yg<1 & Yg>-1) Yg =0;
    ang += Yg*Del;	
		htim->Instance->CNT = 0;	
//	  
	 sprintf(string, "init_Angle = %f \t" , ang);
	 HAL_UART_Transmit(&huart1,(uint8_t *)&string ,sizeof(string) ,0xFFFF );
	
		
		 pid_error = ang-pid_setpoint;
		 if(pid >10 || pid < -10)pid_error += pid * 0.015 ;
	
		 float proportional = pid_error * P;
		
		 static float integral = 0;
		 integral += pid_error * I;
		 if(integral >  400) integral = 400; // limit wind-up
		 if(integral < -400) integral = -400;

		 
		 static float previous_error = 0;
		 float derivative = (pid_error - previous_error) * D;
		 previous_error = pid_error;
		 pid = proportional+derivative+integral;
		 if(pid > 400) pid = 400;
		 if(pid < -400)pid= -400;
		 
		 sprintf(string, "pid = %f \t" ,pid );
		 HAL_UART_Transmit(&huart1,(uint8_t *)&string ,sizeof(string) ,0xFFFF );
		 
		 if(pid <10 && pid>-10) pid =0;//Create a dead-band to stop the motors when the robot is balanced
		 
		 if(ang >15 || ang <-15 )
		 {
			 pid = 0;
			 integral =0;  //Reset the I-controller memory
		 }
		 
		 if(pid < 0)
		 {
				pid_output_left  = -1*pid; //Left pid will positive so speed increases
				pid_output_right = pid; //In Right pid will bee negative so Speed Decreases
			
		 }
		 
		 else if(pid > 0)
		 {
			 	pid_output_left  =  -1*pid;//Left pid will negative so speed Decreases
				pid_output_right =  pid; //In Right pid will be Positive so Speed Decreases
		
		 }
		 
		 else 
		 {
				pid_output_left = 0;
				pid_output_right = 0;
		 }
		 
		  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
		 else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;
	

		 if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
		 else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;
		 
		  if(pid_output_left > 0)(*left_motor) = 400 - pid_output_left;
		 else if(pid_output_left < 0)(*left_motor) = -400 - pid_output_left;
		 else (*left_motor) = 0;

		 if(pid_output_right > 0)(*right_motor) = 400 - pid_output_right;
		 else if(pid_output_right < 0)(*right_motor) = -400 - pid_output_right;
		 else (*right_motor) = 0;
		 
}
*/
