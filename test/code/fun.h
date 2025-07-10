#ifndef _fun_h
#define _fun_h

#include "stm32h750xx.h"


void jxbID0_action_angle(int angle);
void jxbID1_action_angle(int angle);
void jxbID2_action_angle(int angle);
void jxbID3_action_angle();
void jxbID4_action_angle();
void jxbID5_action_angle(int angle);
void jxbID6_action_angle(int angle);
void jxbID7_action_angle(int angle);



void jxb_action_PWM(int ID,int  PWM);
void jxb_catchwire1(void);
void jxb_catchwire2(void);
void screen(void);




extern double djjd[7];
void SF(int coordine);
double angle_COS_degrees(double a, double b, double c) ;

void process_data(int data);

#endif


