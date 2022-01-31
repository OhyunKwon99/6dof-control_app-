///fextern const double PI  ;
///fextern const double c  ;
///fextern const double half_value ;
const double PI = 3.141592 ;
const double c = 0.29296875 ;
const double half_value = 0.29296875/2 ;


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "6dof-xel.h"
#include "round_function.h"
#include "stm32f4xx_hal.h"

extern int cnt ;

///angle
unsigned char angle_1H = 0x01;   // joint angle value
unsigned char angle_1L = 0xff;   // joint angle value
unsigned char angle_4H = 0x01;  // joint angle value
unsigned char angle_4L = 0xff;  // joint angle value
unsigned char angle_5H = 0x01;   // joint angle value
unsigned char angle_5L = 0xff;   // joint angle value
unsigned char angle_6H = 0x01;   // joint angle value
unsigned char angle_6L = 0xff;   // joint angle value

///speed
unsigned char speed_1H = 0x03;   // angular speed value
unsigned char speed_1L = 0xff;   // angular speed value
unsigned char speed_4H = 0x03;  // angular speed value
unsigned char speed_4L = 0x50;  // angular speed value
unsigned char speed_5H = 0x01;   // angular speed value
unsigned char speed_5L = 0x0f;   // angular speed value
unsigned char speed_6H = 0x03;   // angular speed value
unsigned char speed_6L = 0x50;   // angular speed value
uint8_t buffrx[1];

// dynamixel data/////////////////////
unsigned char bufftx[28] = 
{ 0xff, 0xff, 0xfe, 0x18, 0x83, 0x1e, 0x04,  //LENGHT (D+1)*N+4
  0x01, 0xff, 0x01, 0x60, 0x02, //0x01 ID     // 0xff [8]  0x01 [9]  pos     // 0x50 [10] 0x01 [11] speed
  0x04, 0xff, 0x01, 0x60, 0x02, //0x04 ID     // 0xff [13] 0x01 [14] pos     // 0x60 [15] 0x03 [16] speed
  0x05, 0xff, 0x01, 0x60, 0x02, //0x05 ID     // 0xff [18] 0x01 [19] pos     // 0x60 [20] 0x03 [21] speed
  0x06, 0xff, 0x01, 0x60, 0x02, //0x06 ID     // 0xff [23] 0x01 [24] pos     // 0x60 [25] 0x03 [26] speed 
  !(0xfe + 0x1d + 0x83 + 0x1e + 0x04      //check sum    
    + 0x01 + 0xff + 0x01 + 0x60 + 0x02    //check sum 
    + 0x04 + 0xff + 0x01 + 0x60 + 0x02    //check sum 
    + 0x05 + 0xff + 0x01 + 0x60 + 0x02    //check sum 
    + 0x06 + 0xff + 0x01 + 0x60 + 0x02 )  //check sum 
};

///////////////////////////////dynamixel hex data
double hex_1, hex_2, hex_3, hex_4 ;          // dynamixel hex data 

unsigned char 
 hex_1_L, hex_1_H,                // dynamixel data 
 hex_2_L, hex_2_H,                             // dynamixel data 
 hex_3_L, hex_3_H,                             // dynamixel data 
 hex_4_L, hex_4_H ;                            // dynamixel data 

double re_1, re_2, re_3, re_4 ;                  // remainder 


////////////////////////////////link length parameters

//default link length
float a_0 = 0.0, a_1 = 0.0, a_2 = 17.0, a_3 =0.0, a_4 = 0.0, a_5 = 0.0 , a_6 = 5.0;     

 //link twist 
float alpha_0 = 0.0, alpha_1 = 90.0, alpha_2 = 0.0, alpha_3 = 0.0, alpha_4 = 0.0; 

// joint distanse 
float d_1 = 0.0, d_2 = 0.0, d_3 = 0.0, d_4 = 12.77 ;                 

// joint angle
double temp_1=0.0, temp_2=0.0, temp_3=0.0, temp_4=0.0, temp_5=0.0, temp_6=0.0; 
double theta_1 = 0.0, theta_2 = 0.0, theta_3 = 0.0, theta_4 = 0.0, theta_5 = 0.0, theta_6=0.0;
double theta_ref1=90.0, theta_ref2=90.0, theta_ref3, theta = 0.0;   
double t1,t2,t3,t4,t5,t6, g ; //theta temp


//////////////////////////////////angle pos calculate parameter

//second joint
float beta, alpha ;       // second joint angle parameter 
float c_alpha, s_alpha ;  // second joint angle parameter 
double angle_sum;        

//check parameters
double K, ro ;
double check1, check2;

//theta_2 parameter
double s_1, c_1, s_3, c_3, A ,B_1, B_2, B_3, C_1;

//theta_4 parameter
double c_2, s_2, c_23, s_23 ;

//theta_5 parameter
double c_4, s_4 ;

//theta_6 parameter
double c_5, s_5, c_6, s_6 ;
double t_6temp1,t_6temp2 ;

double s_1t, c_1t, s_2t, c_2t, s_3t, c_3t, s_23t, c_23t, s_4t, s_5t, s_6t, c_4t, c_5t, c_6t ;


////////////////////////////// additional IK param
float al4 = -90.0, al5 = 90.0, a4 = 0.0 , d4 = 4.2 , a5 = 2.7, d6 = 8.8;


//////////////////////////////// endeffector position state
float r11 , r12 , r13 , r21 , r22 , r23 , r31 , r32 , r33  ; // ee matrix pos
float px = 24.0 , py = 0.0 , pz = 10.0; //default ee position                  
float px_c = 0.0, py_c = 0.0, pz_c = 0.0 ;  //error ee position                
float px_n = 0.0, py_n = 0.0, pz_n = 0.0 ;  //calibrate ee position


///////////////////////////////// round function parameters

uint16_t low ;               // round function parameter 
double p_point, m_point ;    // round function parameter
int mid;
uint8_t H = 0, L = 0;


////////////////////////////// mode parameter

extern bool mode ; //mode indicater


////////////////////////////// function 

float deg2rad(float degree)  //angle tf
{
  float radius ;
  radius = degree*PI/180.0;
  return radius ;
}
float rad2deg(float radius)  //angle tf
{
  float degree ;
  degree = radius*180.0/PI ;
  return degree;
}

float cosd(float degree)   //deg func
{
  float radius ;
  radius = degree*PI/180.0;
  return cos(radius) ;
}
float sind(float degree)   //deg func
{
  float radius ;
  radius = degree/180*PI;
  return sin(radius) ;
}

float atan2d(float s, float c)   //deg func
{
  float degree ;
  degree = atan2(s,c)*180.0/PI;
  return degree ;
}
//void endeff_set(float angle)   //endeffector initialize
//{
//  r11 = 1.0, r12 = 0.0 , r13 = 0.0,
//  r21 = 0.0, r22 =cosd(angle), r23 = -sind(angle),
//  r31 = 0.0, r32 = sind(angle), r33 = cosd(angle);
//  //px = 15.0 , py = 0.0 , pz = 15.0 ;
//}
void endeff_set(double angle1, double angle2, double angle3)   //endeffector initialize
{
  //r11 = cosd(angle1)*cosd(angle2), r12 = -sind(angle2), r13 = cosd(angle2)*sind(angle1), r14 = px*cosd(angle1)*cosd(angle2) - py*sind(angle2) + pz*cosd(angle2)*sind(angle1), 
  //r21 = cosd(angle1)*sind(angle2), r22 = cosd(angle2), r23 = sind(angle1)*sind(angle2), r24 = py*cosd(angle2) + px*cosd(angle1)*sind(angle2) + pz*sind(angle1)*sind(angle2), 
  //r31 = -sind(angle1), r32 = 0,r33 = cosd(angle1),r34 = pz*cosd(angle1) - px*sind(angle1), 
  //r41 =  0,r42 = 0,r43 = 0,r44 = 1
  
  //r11 = cosd(angle1)*cosd(angle2), r12 = -cosd(angle1)*sind(angle2), r13 = sind(angle1),
  //r21 = sind(angle2), r22 = cosd(angle2) , r23 = 0.0,
  //r31 = -cosd(angle2)*sind(angle1), r32 = sind(angle1)*sind(angle2), r33 = cosd(angle1);
  ////px = 15.0 , py = 0.0 , pz = 15.0 ;  //////////prismatic //default

  ///r11 = cosd(angle1)*sind(angle2)*sind(angle3) - cosd(angle3)*sind(angle1),    r12 = sind(angle1)*sind(angle3) + cosd(angle1)*cosd(angle3)*sind(angle2), r13 = cosd(angle1)*cosd(angle2),
  ///r21 = - cosd(angle1)*cosd(angle3) - sind(angle1)*sind(angle2)*sind(angle3),  r22 = cosd(angle1)*sind(angle3) - cosd(angle3)*sind(angle1)*sind(angle2), r23 = -cosd(angle2)*sind(angle1),
  ///r31 = -cosd(angle2)*sind(angle3),                                            r32 = -cosd(angle2)*cosd(angle3),                                         r33 = sind(angle2);
  /////px = 15.0 , py = 0.0 , pz = 15.0 ;
  r11 = - cosd(angle3)*sind(angle1) - cosd(angle1)*cosd(angle2)*sind(angle3), r12 = cosd(angle1)*sind(angle2),  r13 = cosd(angle1)*cosd(angle2)*cosd(angle3) - sind(angle1)*sind(angle3), 
  r21 = cosd(angle2)*sind(angle1)*sind(angle3) - cosd(angle1)*cosd(angle3),   r22 = -sind(angle1)*sind(angle2), r23 = - cosd(angle1)*sind(angle3) - cosd(angle2)*cosd(angle3)*sind(angle1),                                                  -sind(angle2)*sind(angle3),                          -cosd(angle2),                                                                                cosd(angle3)*sind(angle2),
  r31 = -sind(angle2)*sind(angle3),                                           r32 = -cosd(angle2),               r33 = cosd(angle3)*sind(angle2) ;
  
  
  
  
  
}
//void round1()    // round off func
//{
//  low = (int)(hex_1);
//  mid = (int)((hex_1-(double)(low))/half_value);
//  H = (uint8_t)(hex_1/256);
//  L = (uint8_t)(hex_1-256*H);
//  switch (mid)
//  {
//  case 0:
//    break;
//    
//  case 1:
//    if(L != 255)
//    {
//      L = L + 1;
//    }
//    else
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;
//    
//  case 2:
//    if(L != 255)
//    {
//      L = L + 1;
//    }
//    else
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;
//    
//  case 3:
//    if((L != 255)&&(L != 254)) 
//    { 
//     L = L + 2; 
//    } 
//    else if(L == 255)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else
//    {
//      H = H + 1; 
//      L = 0;
//    }
//    break ; 
//    
//  case 4:
//    if((L != 255)&&(L != 254)) 
//    { 
//     L = L + 2; 
//    } 
//    else if(L == 255)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else
//    {
//      H = H + 1; 
//      L = 0;
//    }
//    break ;  
//    
//  case 5:        
//    if((L != 255)&&(L != 254)&&(L != 253))      
//    {      
//      L = L + 3;      
//    }      
//    else if(L == 255)
//    { 
//      H = H + 1;
//      L = 2; 
//    }
//    else if(L == 254)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else 
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ; 
//         
//  case 6:        
//    if((L != 255)&&(L != 254)&&(L != 253))      
//    {      
//      L = L + 3;      
//    }      
//    else if(L == 255)
//    { 
//      H = H + 1;
//      L = 2; 
//    }
//    else if(L == 254)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else 
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;      
//    
//  }
//  hex_1_H = (H);
//  hex_1_L = (L);
//  
//}
//void round2()    // round off func
//{
//  low = (int)(hex_2);
//  mid = (int)((hex_2-(double)(low))/half_value);
//  H = (uint8_t)(hex_2/256);
//  L = (uint8_t)(hex_2-256*H);
//  switch (mid)
//  {
//  case 0:
//    break;
//    
//  case 1:
//    if(L != 255)
//    {
//      L = L + 1;
//    }
//    else
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;
//    
//  case 2:
//    if(L != 255)
//    {
//      L = L + 1;
//    }
//    else
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;
//    
//  case 3:
//    if((L != 255)&&(L != 254)) 
//    { 
//     L = L + 2; 
//    } 
//    else if(L == 255)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else
//    {
//      H = H + 1; 
//      L = 0;
//    }
//    break ; 
//    
//  case 4:
//    if((L != 255)&&(L != 254)) 
//    { 
//     L = L + 2; 
//    } 
//    else if(L == 255)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else
//    {
//      H = H + 1; 
//      L = 0;
//    }
//    break ;  
//    
//  case 5:        
//    if((L != 255)&&(L != 254)&&(L != 253))      
//    {      
//      L = L + 3;      
//    }      
//    else if(L == 255)
//    { 
//      H = H + 1;
//      L = 2; 
//    }
//    else if(L == 254)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else 
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ; 
//         
//  case 6:        
//    if((L != 255)&&(L != 254)&&(L != 253))      
//    {      
//      L = L + 3;      
//    }      
//    else if(L == 255)
//    { 
//      H = H + 1;
//      L = 2; 
//    }
//    else if(L == 254)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else 
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;      
//    
//  }
//  hex_2_H = (H);
//  hex_2_L = (L);
//}
//void round3()    // round off func
//{
//  low = (int)(hex_3);
//  mid = (int)((hex_3-(double)(low))/half_value);
//  H = (uint8_t)(hex_3/256);
//  L = (uint8_t)(hex_3-256*H);
//  switch (mid)
//  {
//  case 0:
//    break;
//    
//  case 1:
//    if(L != 255)
//    {
//      L = L + 1;
//    }
//    else
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;
//    
//  case 2:
//    if(L != 255)
//    {
//      L = L + 1;
//    }
//    else
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;
//    
//  case 3:
//    if((L != 255)&&(L != 254)) 
//    { 
//     L = L + 2; 
//    } 
//    else if(L == 255)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else
//    {
//      H = H + 1; 
//      L = 0;
//    }
//    break ; 
//    
//  case 4:
//    if((L != 255)&&(L != 254)) 
//    { 
//     L = L + 2; 
//    } 
//    else if(L == 255)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else
//    {
//      H = H + 1; 
//      L = 0;
//    }
//    break ;  
//    
//  case 5:        
//    if((L != 255)&&(L != 254)&&(L != 253))      
//    {      
//      L = L + 3;      
//    }      
//    else if(L == 255)
//    { 
//      H = H + 1;
//      L = 2; 
//    }
//    else if(L == 254)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else 
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ; 
//         
//  case 6:        
//    if((L != 255)&&(L != 254)&&(L != 253))      
//    {      
//      L = L + 3;      
//    }      
//    else if(L == 255)
//    { 
//      H = H + 1;
//      L = 2; 
//    }
//    else if(L == 254)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else 
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;      
//    
//  }
//  hex_3_H = (H);
//  hex_3_L = (L);
//}
//void round4()    // round off func
//{
//  low = (int)(hex_4);
//  mid = (int)((hex_4-(double)(low))/half_value);
//  H = (uint8_t)(hex_4/256);
//  L = (uint8_t)(hex_4-256*H);
//  switch (mid)
//  {
//  case 0:
//    break;
//    
//  case 1:
//    if(L != 255)
//    {
//      L = L + 1;
//    }
//    else
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;
//    
//  case 2:
//    if(L != 255)
//    {
//      L = L + 1;
//    }
//    else
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;
//    
//  case 3:
//    if((L != 255)&&(L != 254)) 
//    { 
//     L = L + 2; 
//    } 
//    else if(L == 255)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else
//    {
//      H = H + 1; 
//      L = 0;
//    }
//    break ; 
//    
//  case 4:
//    if((L != 255)&&(L != 254)) 
//    { 
//     L = L + 2; 
//    } 
//    else if(L == 255)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else
//    {
//      H = H + 1; 
//      L = 0;
//    }
//    break ;  
//    
//  case 5:        
//    if((L != 255)&&(L != 254)&&(L != 253))      
//    {      
//      L = L + 3;      
//    }      
//    else if(L == 255)
//    { 
//      H = H + 1;
//      L = 2; 
//    }
//    else if(L == 254)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else 
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ; 
//         
//  case 6:        
//    if((L != 255)&&(L != 254)&&(L != 253))      
//    {      
//      L = L + 3;      
//    }      
//    else if(L == 255)
//    { 
//      H = H + 1;
//      L = 2; 
//    }
//    else if(L == 254)
//    { 
//      H = H + 1; 
//      L = 1; 
//    }
//    else 
//    {
//      H = H + 1;
//      L = 0;
//    }
//    break ;      
//    
//  }
//  hex_4_H = (H);
//  hex_4_L = (L);
//}
//
void seperate(double hex_1, double hex_2, double hex_3, double hex_4)  //seperate by priority bits
{
  hex_1_H = (unsigned char)((uint8_t)(hex_1/256)) ; //  high priority bit
  hex_1_L = (unsigned char)((uint8_t)(hex_1-(double)(256*hex_1_H))) ; //  low 
                       
  hex_2_H = (unsigned char)((uint8_t)(hex_2/256)) ; //  high priority bit
  hex_2_L = (unsigned char)((uint8_t)(hex_2-(double)(256*hex_2_H))) ; //  low
                    
  hex_3_H = (unsigned char)((uint8_t)(hex_3/256)) ; //  high priority bit
  hex_3_L = (unsigned char)((uint8_t)(hex_3-(double)(256*hex_3_H))) ; //  low
                                    
  hex_4_H = (unsigned char)((uint8_t)(hex_4/256)) ; //  high priority bit
  hex_4_L = (unsigned char)((uint8_t)(hex_4-(double)(256*hex_4_H))) ; //  low
  round2(); 
  round3();
  round4();
  round1(); 
}

void dec2hex(double dec_1, double dec_2, double dec_3, double dec_4)  //change data form
{ 
  // c = (0~300 deg) to (0x0000~0x03ff) ratio  
  hex_1 = ((dec_1+150.0)/c) ;
  hex_2 = ((dec_2+150.0)/c) ;
  hex_3 = ((dec_3+150.0)/c) ;
  hex_4 = ((dec_4+150.0)/c) ;
  
  seperate(hex_1, hex_2, hex_3, hex_4);
}

double angle_calibrate(double angle)  // angle limit func
{ //joint angle range (-150 ~150)
  double temp = fmod(angle,360.0);
  if(fabs(temp)>=150.0)
  {
    if(angle<0.0)
    {
      temp = -150.0 ;
    }
    else
    {
      temp = 150.0 ;
    }
  }
  
  return temp ;
}

double angle_calibrate_5(double angle)  // angle limit func
{ //joint angle range (-150 ~150)
  if(fabs(angle)>=100.0)
  {
    if(angle<0.0)
    {
      angle = -100.0 ;
    }
    else
    {
      angle = 100.0 ;
    }
  }
  
  return angle ;
}

double theta2limit(double input)
{
  if(input>130.0)
  {
    
    input = 130.0;
  }
  else if(input<=0.0)
  {
    
    input = 0.0;
  } 
  else
  {
    mode = 0;
  }
  
  return input;
}
double theta3limit(double input)
{
  if(input>140.0)
  {
    
    input = 140.0;
  }
  else if(input<-140.0)
  {
    
    input = -140.0;
  }
  else
  {
    mode = 0;
  }
  
  return input;
}

void pos_calibrate()
{
  
  s_1t = sind(temp_1);
  c_1t = cosd(temp_1);
  s_2t = sind(temp_2);
  c_2t = cosd(temp_2);
  s_3t = sind(temp_3);
  c_3t = cosd(temp_3);
  s_23t = sind(temp_2+temp_3);
  c_23t = cosd(temp_2+temp_3);
  
  s_4t = sind(t4);
  c_4t = cosd(t4);
  s_5t = sind(t5);
  c_5t = cosd(t5);
  s_6t = sind(t6);
  c_6t = cosd(t6);
  
  
  px_c = d6*(s_5t*(s_1t*s_4t - s_23t*c_1t*c_4t) + c_23t*c_1t*c_5t) + d4*c_23t*c_1t ;
  py_c = d4*c_23t*s_1t - d6*(s_5t*(c_1t*s_4t + s_23t*c_4t*s_1t) - c_23t*c_5t*s_1t) ;
  pz_c = d4*s_23t + d6*(s_23t*c_5t + c_23t*c_4t*s_5t) ;
  
  //px_c = d4*s_1t - a5*(s_5t*(c_23t*c_1t*s_4t + s_23t*c_1t*c_4t) - c_5t*s_1t) - d6*(s_5t*(c_23t*c_1t*s_4t + s_23t*c_1t*c_4t) - c_5t*s_1t);
  //py_c = - d4*c_1t - a5*(s_5t*(c_23t*s_1t*s_4t + s_23t*c_4t*s_1t) + c_1t*c_5t) - d6*(s_5t*(c_23t*s_1t*s_4t + s_23t*c_4t*s_1t) + c_1t*c_5t);
  //pz_c = a5*s_5t*(c_23t*c_4t - s_23t*s_4t) + d6*s_5t*(c_23t*c_4t - s_23t*s_4t);
 
  //px_c = d4*s_23t*s_1t - d6*(s_5t*(c_1t*s_4t + c_23t*c_4t*s_1t) - s_23t*c_5t*s_1t) - a5*(s_5t*(c_1t*s_4t + c_23t*c_4t*s_1t) - s_23t*c_5t*s_1t);
  //py_c = - a5*(s_5t*(s_1t*s_4t - c_23t*c_1t*c_4t) + s_23t*c_1t*c_5t) - d6*(s_5t*(s_1t*s_4t - c_23t*c_1t*c_4t) + s_23t*c_1t*c_5t) - d4*s_23t*c_1t;
  //pz_c = d4*c_23t + a5*(c_23t*c_5t + s_23t*c_4t*s_5t) + d6*(c_23t*c_5t + s_23t*c_4t*s_5t) ;

  
  //px_c = a_6*(cosd(t4)*sind(t6) - cosd(t6)*sind(t4)*sind(t5));
  //py_c = a_6*(sind(t4)*sind(t6) + cosd(t4)*cosd(t6)*sind(t5));
  //pz_c = d_4 + a_6*cosd(t5)*cosd(t6);
    
    
  //px_c = a6 + a5*cosd(t6) + d4*(sind(al4)*(cosd(t5)*cosd(t6) - cosd(al5)*sind(t5)*sind(t6)) + cosd(al4)*sind(al5)*sind(t6));
  //py_c = a5*sind(t6) + d4*(sind(al4)*(cosd(t5)*sind(t6) + cosd(al5)*cosd(t6)*sind(t5)) - cosd(al4)*sind(al5)*cosd(t6));
  //pz_c = d4*(cosd(al4)*cosd(al5) + sind(al4)*sind(al5)*sind(t5));
  px_n = px - px_c;
  py_n = py - py_c;
  pz_n = pz - pz_c;

  cal_target_val(px_n, py_n, pz_n);
  
  t1 = angle_calibrate(temp_1) ; // xel
  t2 = theta2limit(temp_2);  //motor
  t3 = theta3limit(temp_3);  //motor
 /*
  if(check1<1)
  {
    //////////// theta_1
    t1 = atan2d(py_n, px_n) ;//- atan2d(d_3, sqrt( pow(px,2.0) + pow(py,2.0) - pow(d_3,2.0)) );
    c_1 = cosd(temp_1);
    s_1 = sind(temp_1);
      
      
    //////////// theta_3
    c_3 = ((pow(px_n,2.0)+pow(py_n,2.0)+pow(pz_n,2.0))-(pow(a_2,2.0)+pow(d_4,2.0)))/(2*(a_2)*(d_4));
    s_3 = -sqrt(1-pow(c_3,2.0));
    t3 = atan2d(s_3, c_3);
    
    
    //////////// theta_2
    A = (px_n*c_1 + py_n*s_1);
    B_1 = c_3*(A+pz_n);
    B_2 = (A*s_3 - pz_n*c_3);
    B_3 = (A*c_3 + pz_n*s_3);
    //s_2 = ((d_4+(a_2*c_3))*(c_3*pz - s_3*A)+(a_2*s_3)*(c_3*A + s_3*pz)) / ((c_3*A + s_3*pz)*(c_3*A + s_3*pz) +pow(c_3*pz - s_3*A,2.0));
    s_2 = -(d_4 + a_2*c_3 - (a_2*s_3*(B_3))/(B_2))/((B_2)*(pow((B_3),2.0)/pow((B_2),2.0) + 1));
    c_2 = -sqrt(1-pow(s_2,2.0));
    t2 = atan2d(s_2,c_2);
    
    t1 = angle_calibrate(t1) ; // xel
    t2 = theta2limit(t2);  //motor
    t3 = theta3limit(t3);  //motor
  }
  */
}

void cal_target_val(float px, float py, float pz)  // inverse kinematics
{
  K = (pow(px,2.0) + pow(py,2.0) + pow(pz,2.0) - pow(a_2,2.0) - pow(a_3,2.0) - pow(d_3,2.0) - pow(d_4,2.0))/(2*a_2*d_4) ;
  check1 = fabs(K/sqrt(pow(a_3,2.0)+pow(d_4,2.0)));
 
  if(check1<1)
  {
    
    //////////// theta_1
    
    temp_1 = atan2d(py, px) ;//- atan2d(d_3, sqrt( pow(px,2.0) + pow(py,2.0) - pow(d_3,2.0)) );
    c_1 = cosd(temp_1);
    s_1 = sind(temp_1);
    
    
    //////////// theta_3
    
    c_3 = ((pow(px,2.0)+pow(py,2.0)+pow(pz,2.0))-(pow(a_2,2.0)+pow(d_4,2.0)))/(2*(a_2)*(d_4));
    s_3 = -sqrt(1-pow(c_3,2.0)); //angle range -PI~0 ==> -sign
    
    temp_3 = atan2d(s_3, c_3);
    //c_3 = cosd(temp_3);unnecessary
    //s_2 = sind(temp_3);unnecessary
    
    
    //////////// theta_2
    
    A = (px*c_1 + py*s_1);
    B_1 = c_3*(A+pz);
    B_2 = (A*s_3 - pz*c_3);
    B_3 = (A*c_3 + pz*s_3);
    {
    //s_2 = ((d_4+(a_2*c_3))*(c_3*pz - s_3*A)+(a_2*s_3)*(c_3*A + s_3*pz)) / ((c_3*A + s_3*pz)*(c_3*A + s_3*pz) +pow(c_3*pz - s_3*A,2.0));
    //s_2 = -(d_4 + a_2*c_3 - (a_2*s_3*(B_3))/(B_2))/((B_2)*(pow((B_3),2.0)/pow((B_2),2.0) + 1));
    //c_2 = (A*a_2*pow(c_3,2.0) + A*d_4*c_3 + A*a_2*pow(s_3,2.0) + d_4*pz*s_3)/((pow(A,2.0) + pow(pz,2.0))*(pow(c_3,2.0) + pow(s_3,2.0)));
    //c_2 = (d_4 + c_3*(a_2 - (a_2*pz*s_3)/(pz*s_3 + c_3*(c_1*px + py*s_1))) + (a_2*pow(s_3,2.0)*(c_1*px + py*s_1))/(pz*s_3 + c_3*(c_1*px + py*s_1)))/(c_3*(c_1*px + py*s_1 + (pz*(c_3*pz - s_3*(c_1*px + py*s_1)))/(pz*s_3 + c_3*(c_1*px + py*s_1))) + s_3*(pz - ((c_3*pz - s_3*(c_1*px + py*s_1))*(c_1*px + py*s_1))/(pz*s_3 + c_3*(c_1*px + py*s_1))));
    }
    c_2 = (d_4 + c_3*(a_2 - (a_2*pz*s_3)/(B_3)) + (A*a_2*pow(s_3,2.0))/(B_3))/(c_3*(A - (pz*(B_2))/(B_3)) + s_3*(pz + (A*(B_2))/(B_3)));
    s_2 = sqrt(1-pow(c_2,2.0)); //angle range 0~PI ==> +sigN
    
    temp_2 = atan2d(s_2,c_2);
    //s_2 = sind(temp_2);
    //c_2 = cosd(temp_2); 
    
    
    //////////// theta_5
    
    
    //c_2 = cosd(temp_2);
    //s_2 = sind(temp_2);
  
    c_23 = cosd(temp_2+temp_3);
    s_23 = sind(temp_2+temp_3);
    
    if(temp_2+temp_3<=0)
    {
      s_23 = -s_23;
      check2=1;
    }
    else
    {
      check2=0;
    }
    /**/
    
    /////////////theta_5
    
    
    C_1 = r13*c_1 + r23*s_1 ;
    c_5 = c_3*(c_2*(C_1) + r33*s_2) - s_3*(s_2*(C_1) - r33*c_2);
    s_5 = sqrt(1-pow(c_5,2.0)); //angle range 0~PI ==> +sign
    temp_5 = atan2d(s_5,c_5); ///temperary theta_5 data for following if logic
    
    
     //////////// theta_4
    
    if(temp_5 != 0.0)
    {
      if(theta_5 > 0.0) //py  ==>  theta_$ range cal needed@@@@@@@@@
      {
        s_4 = (r13*s_1 - r23*c_1) ;
        c_4 = -c_3*(s_2*(C_1)-r33*c_2)-s_3*(c_2*(C_1)+r33*s_2);
        temp_4 = atan2d(s_4,c_4);
        t4 = angle_calibrate(temp_4) ; // xel
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
      }
      else
      {
        s_4 = (r13*s_1 - r23*c_1) ;
        c_4 = -c_3*(s_2*(C_1)-r33*c_2)-s_3*(c_2*(C_1)+r33*s_2);
        temp_4 = atan2d(s_4,c_4);
        t4 = angle_calibrate(temp_4) ; // xel
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
      }
    }
    else
    {
      temp_5 = 0.01;
      s_4 = (r13*s_1 - r23*c_1) ;
      c_4 = -c_3*(s_2*(C_1)-r33*c_2)-s_3*(c_2*(C_1)+r33*s_2);
      temp_4 = atan2d(s_4,c_4);
    }
    
    //s_5 = (s_4*(r23*c_1-r13*s_1)-c_4*((c_3*(s_2*(r13*c_1+r23*s_1)-r33*c_2))+s_3*(c_2*(r13*c_1+r23*s_1)+r33*s_2)));
    //////////////2D theta sum
    
    {
    //theta_4 = atan2d(-(r13*s_1 - r23*c_1), fabs(r12*c_1*c_23 + r23*s_1*c_23 -r33*s_23) ) ;
    
    
      //theta_3
    //float s_3, c_3 ;
    //c_3 = (pow(px,2.0)+pow(pz,2.0)-pow(a_2,2.0)-pow(a_3,2.0))/(2*a_2*a_3) ;
    //s_3 = -sqrt(fabs(1-pow(c_3,2.0))) ;
    //theta_3 = atan2d(s_3,c_3) ;
    
    //theta_4
    //theta_4 = atan2d(r31,r32)-theta_2-theta_3 ;
    
    //angle calibrate
    
    
    //////////// theta_5
    //c_4 = cosd(theta_4);
    //s_4 = sind(theta_4);
    
    //theta_5 = atan2d( -r12*(c_1*c_23*c_4 + s_1*s_4) -(r23*(s_1*c_23*c_4 - c_1*s_4)) +r33*s_23*c_4, -(r13*c_1*s_23-r23*s_1*s_23-r33*c_23));
   //if(theta_5<0)
   //{
   //  theta_4=theta_4-180.0;
   //}
    //////////// theta_6
    //c_5 = cosd(theta_5);
    //s_5 = sind(theta_5);
    }
   
    //////////////theta_6
    t_6temp1 = (r11*c_1+r21*s_1);
    t_6temp2 = (r12*c_1+r22*s_1);
    {
    //c_6 = c_5*(s_4*(r21*c_1 - r11*s_1) + c_4*(c_3*(c_2*(t_6temp)+r31*s_2) - s_3*(s_2*(t_6temp) - r31*c_2))) - s_5*(c_3*(s_2*(t_6temp)-r31*c_2)+s_3*(c_2*(t_6temp)+r31*s_2));
    //s_6 = s_4*(c_3*(c_2*(t_6temp)+r31*s_2)-s_3*(s_2*(t_6temp)-r31*c_2)) - c_4*(r21*c_1 - r11*s_1);
    // c_6 =-(s_4*(c_3*(s_2*(t_6temp1)-r31*c_2)+s_3*(c_2*(t_6temp1)+r31*s_2))-c_4*(r21*c_1-r11*s_1)); 
    //s_6 =(s_4*(c_3*(s_2*(t_6temp2)-r32*c_2)+s_3*(c_2*(t_6temp2)+r32*s_2))-c_4*(r22*c_1-r12*s_1));
    }
    
    //c_6 = - c_5*(s_4*(r22*c_1 - r12*s_1) + c_4*(c_3*(s_2*(t_6temp2) - r32*c_2) + s_3*(c_2*(t_6temp2) + r32*s_2))) - s_5*(c_3*(c_2*(t_6temp2) + r32*s_2) - s_3*(s_2*(t_6temp2) - r32*c_2)); 
    //s_6 = sqrt(1-pow(c_6,2.0));
    s_6 = s_4*(c_3*(s_2*(r12*c_1 + r22*s_1) - r32*c_2) + s_3*(c_2*(r12*c_1 + r22*s_1) + r32*s_2)) - c_4*(r22*c_1 - r12*s_1);
    c_6 = - c_5*(s_4*(r22*c_1 - r12*s_1) + c_4*(c_3*(s_2*(r12*c_1 + r22*s_1) - r32*c_2) + s_3*(c_2*(r12*c_1 + r22*s_1) + r32*s_2))) - s_5*(c_3*(c_2*(r12*c_1 + r22*s_1) + r32*s_2) - s_3*(s_2*(r12*c_1 + r22*s_1) - r32*c_2));
    
    //c_6 = sqrt(1-pow(s_6,2.0));
    temp_6 = atan2d(s_6, c_6);
    
    //t1 = angle_calibrate(temp_1) ; // xel
    //t2 = theta2limit(temp_2);  //motor
    //t3 = theta3limit(temp_3);  //motor
    //t4 = angle_calibrate(temp_4) ; // xel
    t5 = angle_calibrate_5(temp_5) ; // xel
    t6 = angle_calibrate(temp_6) ; // xel
    
    // hex_test1 = (unsigned char)(theta_1/c) ;
    // hex_test2 = (unsigned char)(theta_2/c) ;
    // hex_test3 = (unsigned char)(theta_3/c) ;
    // hex_test4 = (unsigned char)(theta_4/c) ;
  }
  
}


void value_assign()  // calculated data assign
{
   angle_1L = hex_1_L ;  
   angle_1H = hex_1_H ;
   
   angle_4L = hex_2_L ;
   angle_4H = hex_2_H ;
   
   angle_5L = hex_3_L ;
   angle_5H = hex_3_H ;
   
   angle_6L = hex_4_L ;
   angle_6H = hex_4_H ;
}


void Packet_value_set()  //assign data to transmit buffer
{
  bufftx[8]  = angle_1L;
  bufftx[9]  = angle_1H;
  bufftx[10] = speed_1L;
  bufftx[11] = speed_1H;
  
  bufftx[13] = angle_4L;
  bufftx[14] = angle_4H;      
  bufftx[15] = speed_4L;
  bufftx[16] = speed_4H;
  
 //bufftx[18] = angle_4L;
 //bufftx[19] = angle_4H;      
 //bufftx[20] = speed_4L;
 //bufftx[21] = speed_4H;     
                                                                               //{ 0xff, 0xff, 0xfe, 0x1d, 0x83, 0x1e, 0x04,
  bufftx[18] = angle_5L;                                                       //  0x00, 0xff, 0x01, 0x50, 0x01, //0x00 ID     // 0xff [8]  0x01 [9]  pos     // 0x50 [10] 0x01 [11] speed
  bufftx[19] = angle_5H;                                                       //  0x01, 0xff, 0x01, 0x60, 0x03, //0x01 ID     // 0xff [13] 0x01 [14] pos     // 0x60 [15] 0x03 [16] speed
  bufftx[20] = speed_5L;                                                       //  0x02, 0xff, 0x01, 0x60, 0x03, //0x02 ID     // 0xff [18] 0x01 [19] pos     // 0x60 [20] 0x03 [21] speed
  bufftx[21] = speed_5H;                                                       //  0x03, 0xff, 0x01, 0x60, 0x03, //0x03 ID     // 0xff [23] 0x01 [24] pos     // 0x60 [25] 0x03 [26] speed
                                                                               //  0x04, 0xff, 0x01, 0x60, 0x03, //0x04 ID     // 0xff [28] 0x01 [29] pos     // 0x60 [30] 0x03 [31] speed
  bufftx[23] = angle_6L;                                                       //  !(0xfe + 0x1d + 0x83 + 0x1e + 0x04      //check sum 
  bufftx[24] = angle_6H;                                                       //    + 0x00 + 0xff + 0x01 + 0x50 + 0x01    //check sum 
  bufftx[25] = speed_6L;                                                       //    + 0x01 + 0xff + 0x01 + 0x60 + 0x03    //check sum 
  bufftx[26] = speed_6H;                                                       //    + 0x02 + 0xff + 0x01 + 0x60 + 0x03    //check sum 
                                                                               //    + 0x03 + 0xff + 0x01 + 0x60 + 0x03    //check sum 
  bufftx[27] =  ~(0xfe + 0x18 + 0x83 + 0x1e + 0x04                             //    + 0x04 + 0xff + 0x01 + 0x60 + 0x03 )  //check sum 
  + 0x01 + angle_1L + angle_1H + speed_1L + speed_1H
  + 0x04 + angle_4L + angle_4H + speed_4L + speed_4H
  + 0x05 + angle_5L + angle_5H + speed_5L + speed_5H
  + 0x06 + angle_6L + angle_6H + speed_6L + speed_6H );
}
