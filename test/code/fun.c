#include "headfile.h"
#include "stm32h750xx.h"

void jxbID0_action_angle(int angle)
{		
	char text[20];
	angle= 500+(angle+45) *((2500-500)/270);
	sprintf (text,"#000P%04dT1000!\r\n",angle);
	HAL_UART_Transmit(&huart2 ,(uint8_t *)text, sizeof (text),50);
	HAL_Delay (500);
}
void jxbID1_action_angle(int angle)
{		
	char text[20];
	angle=500 + (315 - angle ) *((2500-500)/270);
	sprintf (text,"#001P%04dT1000!\r\n",angle);
	HAL_UART_Transmit(&huart2 ,(uint8_t *)text, sizeof (text),50);
	HAL_Delay (500);
}

void jxbID2_action_angle(int angle)
{		
	char text[20];
	angle=500 + (angle-45) *((2500-500)/270)  ;
	sprintf (text,"#002P%04dT1000!\r\n",angle);
	HAL_UART_Transmit(&huart2 ,(uint8_t *)text, sizeof (text),50);
	HAL_Delay (500);
}
void jxbID3_action_angle()
{		
	char text[20];
	sprintf (text,"#003P1600dT1000!\r\n");
	HAL_UART_Transmit(&huart2 ,(uint8_t *)text, sizeof (text),50);
	HAL_Delay (500);
}
void jxbID4_action_angle()
{		
	char text[20];
	sprintf (text,"#004P1800T1000!\r\n");
	HAL_UART_Transmit(&huart2 ,(uint8_t *)text, sizeof (text),50);
	HAL_Delay (500);
}
void jxbID5_action_angle(int angle)
{		
	char text[20];
	angle=500 + (angle-45) *((2500-500)/270)  ;
	sprintf (text,"#005P%04dT1000!\r\n",angle);
	HAL_UART_Transmit(&huart2 ,(uint8_t *)text, sizeof (text),50);
	HAL_Delay (500);
}
void jxbID6_action_angle(int angle)
{		
	char text[20];
	angle=500 + (angle-45) *((2500-500)/270) ;
	sprintf (text,"#006P%04dT1000!\r\n",angle);
	HAL_UART_Transmit(&huart2 ,(uint8_t *)text, sizeof (text),50);
	HAL_Delay (500);
}
void jxbID7_action_angle(int angle)
{		
	char text[20];
	sprintf (text,"#007P%04dT1000!\r\n",angle);
	HAL_UART_Transmit(&huart2 ,(uint8_t *)text, sizeof (text),50);
	HAL_Delay (500);
}





void jxb_action_PWM(int ID,int  PWM)
{		
	char text[20];
	if(PWM<1000)
		{
			sprintf (text,"#00%dP0%dT1000!\r\n",ID,PWM );
			HAL_UART_Transmit(&huart2 ,(uint8_t *)text, sizeof (text),50);
			HAL_Delay (500);
		}
	else 
		{
			sprintf (text,"#00%dP%dT1000!\r\n",ID,PWM );
			HAL_UART_Transmit(&huart2 ,(uint8_t *)text, sizeof (text),50);
			HAL_Delay (500);
		}
			
	
}


void jxb_catchwire1(void)
{
	jxb_action_PWM(0,850);
		HAL_Delay(50);
	jxb_action_PWM(1,1600);
	HAL_Delay(50);
	jxb_action_PWM(2,1300);
	HAL_Delay(50);
	jxb_action_PWM(3,1600);
	HAL_Delay(50);
	jxb_action_PWM(4,1800);
	HAL_Delay(50);
	jxb_action_PWM(5,850);
	HAL_Delay(50);
	jxb_action_PWM(6,1750);
	HAL_Delay(5000);
	jxb_action_PWM(7,2100);
	
	HAL_Delay(5000);
	
	jxb_action_PWM(0,850);
	HAL_Delay(50);
	jxb_action_PWM(1,1500);
	HAL_Delay(50);
	jxb_action_PWM(2,1200);
	HAL_Delay(50);
	jxb_action_PWM(3,1900);
	HAL_Delay(50);
	jxb_action_PWM(4,1750);
	HAL_Delay(50);
	jxb_action_PWM(5,1100);
	HAL_Delay(50);
	jxb_action_PWM(6,1750);
	HAL_Delay(5000);
	jxb_action_PWM(7,1700);
	
	HAL_Delay(5000);
	

	
}

void jxb_catchwire2(void)
{
	jxb_action_PWM(0,150);
	jxb_action_PWM(1,1450);
	jxb_action_PWM(2,1300);
	jxb_action_PWM(3,1600);
	jxb_action_PWM(4,1800);
	jxb_action_PWM(5,850);
	jxb_action_PWM(6,1750);
	HAL_Delay(2000);
	jxb_action_PWM(7,2100);
}








double djjd[7];


#define M_PI 3.1415926
#define h 4
#define X1 10.5
#define X2 13.5
#define X3 15


double angle_COS_degrees(double a, double b, double c) 
{
    double cos_C = (a * a + b * b - c * c) / (2 * a * b);
    double angle_C_radians = acos(cos_C);
    double angle_C_degrees = angle_C_radians * (180.0 / M_PI);
    return angle_C_degrees;
}






void SF(int coordine)
{

    int x ,y , f;
    y = ((coordine % 1000) * 14.5) / 480;  
    x = ((coordine / 1000) * 19.5) / 640;
	
	
	
		double t,theta1,theta2,theta3,angle,theta4,theta5,theta6,theta7,theta8;
    if(x < 18.5)
    {
        angle = (atan(y/(18.5-x)) * (180.0 / M_PI));
        t = sqrt(y*y + (18.5-x)*(18.5-x));
        djjd[0] = angle;
    }
    if(x > 18.5)
    {
        angle = ((M_PI - atan(y/(x-18.5))) * (180.0 / M_PI));
        t = sqrt(y*y + (x-18.5)*(x-18.5));
        djjd[0] = angle;
    }
    if(x == 18.5)
    {
        angle = 90;
        t = y;
        djjd[0] = angle;
    }
    theta1 = angle_COS_degrees(X1,t,X2);
    theta2 = angle_COS_degrees(X1,X2,t);
    theta3 = angle_COS_degrees(X2,t,X1);
    djjd[1] = theta1 + 90;
    djjd[2] = theta2;
    djjd[3] = theta3 + 90;

    f = sqrt(t*t + h*h);
    theta4 = angle_COS_degrees(X1,f,X2);
    theta5 = angle_COS_degrees(X1,X2,f);
    theta6 = angle_COS_degrees(X2,f,X1);
    theta7 = angle_COS_degrees(f,h,t);
    theta8 = angle_COS_degrees(f,t,h);
    djjd[4] = theta7 + theta4;
    djjd[5] = theta5;
    djjd[6] = theta6 + theta8;
}





void process_data(int data)
{
	int color = data/1000000; 
	int y = data % 1000;
  int x = data / 1000 - (color *1000);
//	HAL_UART_Transmit(&huart5,(uint8_t *) &color, 1, 50);
//	HAL_UART_Transmit(&huart5,(uint8_t *) &x,sizeof (x), 50);
//	HAL_UART_Transmit(&huart5,(uint8_t *) &y,sizeof (y), 50);
	
	printf("page2.n0.val=%d\xff\xff\xff",x);
	printf("page2.n1.val=%d\xff\xff\xff",y);
	
	
	if(color ==1)
	{
		printf("page2.t4.val=red\xff\xff\xff" );
		printf("page2.t4.pco=63488\xff\xff\xff" );
	}
	
	if(color ==2)
	{
		printf("page2.t4.val=yellow\xff\xff\xff" );
		printf("page2.t4.pco=65504\xff\xff\xff" );
	}
	
	if(color ==3)
	{
		printf("page2.t4.val=green\xff\xff\xff" );
		printf("page2.t4.pco=2024\xff\xff\xff" );
	}
}







