/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "6dof-xel.h"
#include "round_function.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/////////////////////////// encoder parameters
const double PUSLE2DEG = 0.12; 
const int32_t ENCODER_INIT = 30000;
int32_t Encoder3=30000; 
int32_t pre_Encoder3=30000;
int32_t Encoder5=30000; 
int32_t pre_Encoder5=30000;
int32_t ref_encoder2;

////////////////////////// time parameters
const double TIME = 0.01;//
const double RPT = 1.0;
uint8_t num = 0;
double speed = 0.0 ;
double rpm = 0.0 ;


///////////////////////// PID parameter
double PKp_2, PKi_2=0, PKd_2=0, PKc_2=0, PN_2=0, t ;

double SKp_2, SKi_2=0, SKd_2=0, SKc_2=0, SN_2=0 ;

double CKp_2, CKi_2=0, CKd_2=0, CKc_2=0, CN_2=0 ;

double Kp_3, Ki_3=0, Kd_3=0, N_3=0;

double dead_margin = 55;
///////////////////////// motor control parameter 
int32_t ref_pulse2, err_pulse2, en_pulse2 ; 

double ref_angle2, cur_angle2, err_angle2, err_val_2;

double ref_speed2, cur_speed2, err_speed2, ref_rpm2, cur_rpm2, err_rpm2 ,test;

double ref_cur2, cur_cur2, err_cur2; 

double POp_2 = 0, POi_2 = 0 , POd_2 = 0, pre_POd_2, PWM_out=0, PWM_out2=0, PWM_out3, Pout_2=0;

double SOp_2 = 0, SOi_2 = 0 , SOd_2 = 0, pre_SOd_2, Sout_2=0;

double COp_2 = 0, COi_2 = 0 , COd_2 = 0, pre_COd_2, Cout_2=0;

int32_t ref_pulse3, err_pulse3, en_pulse3 ; 

double ref_angle3, cur_angle3, err_angle3, err_val_3;

double ref_speed3, cur_speed3, err_speed3, ref_rpm3, cur_rpm3, err_rpm3 ;

double Op_3 = 0, Oi_3 = 0 , Od_3 = 0, pre_Od_3, out_3;

double sat_rpm2, sat_rpm3;

const bool MOTOR_ON = 0, MOTOR_OFF = 1;

////////////////////////// state parmameter

bool mode = 1;
bool status =1;
bool status_1=1, status_2=1;

double cc;
double cs;
double cp;
double ra = 5.33;
double la = 2;

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
uint8_t cnt = 0;

extern unsigned char bufftx[28] ;


///////////////////////////joint parameter
extern double theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_ref1, theta_ref2, theta_ref3, theta;   // joint angle
extern double temp_1, temp_2, temp_3, temp_4, temp_5, temp_6; 
extern double t1,t2,t3,t4,t5,t6, g ; //theta temp

//////////////////////////endeffector position state
extern float r11 , r12 , r13 , r21 , r22 , r23 , r31 , r32 , r33; 
///

//////////////////////////// endeffector position state
extern float px  , py  , pz  ;   
extern float px_c, py_c, pz_c;
extern float px_n, py_n, pz_n;
///
/////////////////////////// trejectory, move parameter
double range;
bool sign, dr;                // movement function parameter
double move_cnt = 0;         // movement function parameter
double cnt_speed = 0.1;

extern double angle_sum;        
double theta_sum ,theta_sum1;

bool hold =1 ;

///////////////////////// motor current data
uint32_t adcVal[2];
int32_t temp[2];
double cur_sum;
double mot_cur[2];
double mot_temp[2] = {0,0};
double filter = 0.2;
double move_avg[10];

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void toggle();
void control_gain_init();
void ref_val_init();
void calibrate();
double pwm_limit();
double speed_limit(double input);
void read_encoder();
double wind_up();
double position_control();
void motion_activate();
void move();
void trajectory();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



void toggle()
{
  if(num == 1)
  {
    num = 0;
  }
  else
  {
    num = 1;
  }
}


void ref_val_init()
{
  t = TIME;
  ref_angle2 = 0.0;
  err_angle2 = 0.0;
}

void calibrate()
{
  if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==0)&&(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==0)) // button on
  {
    status=0;
    
    TIM3->CNT = ENCODER_INIT;  //ENCODER_INIT=30000
    TIM5->CNT = 31190;  //ENCODER_INIT=30000
  }
  if(status==0)
  {
    read_encoder();
    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
    htim1.Instance->CCR1 = 65;
   
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
    htim1.Instance->CCR2 = 65;
    
    if(Encoder3>=30750)
    {
      htim1.Instance->CCR1 = 0;
      status_1 = 0;
      status_2 = 0;
    }
    
    if((Encoder5<=29250))
    {
      htim1.Instance->CCR2 = 0;
      status_2 = 0;
    }
    
    if((status_1==0)&&(status_2==0))
    {
      mode = MOTOR_ON;
    }
  }
  
  
}

double pwm_limit(double PWM_out)
{
  double out = fabs(PWM_out);
  if(PWM_out>=0.0)
  {
    if(out>=105.0)
    {
      out = 105.0;
      return out ;
    }
    else
    {
      return out ;
    }
  }
  else
  {
    if(out>=105.0)
    {
      out = 105.0;
      return -out ;
    }
    else
    {
      return -out ;
    }
  }
  
  
}
double pwm_limit2(double PWM_out)
{
  double out = fabs(PWM_out);
  if(PWM_out>=0.0)
  {
    if(out>=100.0)
    {
      out = 100.0;
      return out ;
    }
    else
    {
      return out ;
    }
  }
  else
  {
    if(out>=100.0)
    {
      out = 100.0;
      return -out ;
    }
    else
    {
      return -out ;
    }
  }
  
  
}

double signal_limit(double in)
{
  double out = fabs(in);
  if(in>=0.0)
  {
    if(out>=100.0)
    {
      out = 100.0;
      return out ;
    }
    else
    {
      return out ;
    }
  }
  else
  {
    if(out>=100.0)
    {
      out = 100.0;
      return -out ;
    }
    else
    {
      return -out ;
    }
  }
  
  
}

double rpm_limit(double input)
{
  double rpm = 30;
  if (input >= rpm)
  {
    input = rpm;
    return input ;
  }
  else if (input <= -rpm)
  {
    input = -rpm;
    return input ;
  }
  else
  {
    return input ;
  }
}

double err_limit2(double input)
{
  double val = 50;
  if (input >= val)
  {
    input = val;
    return input ;
  }
  else if (input <= -val)
  {
    input = -val;
    return input ;
  }
  else
  {
    return input ;
  }
}

double err_limit3(double input)
{
  if (input >= 100)
  {
    input = 100;
    return input ;
  }
  else if (input <= -100)
  {
    input = -100;
    return input ;
  }
  else
  {
    return input ;
  }
}

void read_encoder()
{
  pre_Encoder3 = Encoder3;
  Encoder3 = TIM3->CNT;
  pre_Encoder5 = Encoder5;
  Encoder5 = TIM5->CNT;
}



void move()  // movement input 
{
  switch (sign){
  case 0: 
    move_cnt = (move_cnt + cnt_speed); // add cnt
    break ;
  case 1:
    move_cnt = (move_cnt - cnt_speed); // subtract cnt
    break;
  }  
  if(move_cnt<=900.0)
  {
    sign = 0;
    move_cnt=900.0;
  }
  else if(move_cnt>=1700.0)
  {
    sign = 1;
    move_cnt=1700.0;
  }
}

void trajectory()
{ 
  if(theta>=30.0)
  {
    dr = 1;
    theta = 30.0;
  }
  else if(theta<=-30.0)
  {
    dr = 0;
    theta = -30.0;
  }

  switch (dr){
  case 0: 
    theta = (theta + cnt_speed); // add cnt
    break ;
  case 1:
    theta = (theta - cnt_speed); // subtract cnt
    break;
  }
  
  //px = 24;//range*cosd(theta)+19;
  //py = 0;//range*sind(theta);
}
//void trajectory()
//{
//  theta = theta + cnt_speed;
//  if(theta>360.0)
//  {
//    theta = 0.0;
//  }
//  px = range*cosd(theta)+19;
//  py = range*sind(theta);
//}
double deadzone_control(double PWM, double offset)
{ 
  double size = fabs(PWM);
  
  if(size < offset)
  {
    return 0;
  }
  else
  {
    return (size-offset);
  }
}
  
void pwm_dir2()
{
  double val = PWM_out3;
  if(PWM_out3>=0)
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
    htim1.Instance->CCR1 = 0.4*deadzone_control(val,dead_margin)+dead_margin;//(uint16_t)(fabs(PWM_out));
    //htim1.Instance->CCR2 = (uint16_t)pwm_limit(PWM_out2);
  }
  else
    
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
    htim1.Instance->CCR1 = 0.5*deadzone_control(val,dead_margin)+dead_margin-(int)(10*(cos(t2)));//(uint16_t)(fabs(PWM_out));
    //htim1.Instance->CCR2 = (uint16_t)pwm_limit(PWM_out2);
  }
}

//double position_control_2(double theta, int32_t Encoder, int32_t p_Encoder)
//{
//  ref_encoder2 = (int32_t)(theta_2/0.12)+30000;
//  ref_angle2 = t2;
//  cur_angle2 = (double)(PUSLE2DEG*(Encoder-ENCODER_INIT)); 
//  err_angle2 = err_limit2(ref_angle2-(cur_angle2)); //ENCODER_INIT=30000
//  
//  ref_speed2 = err_angle2/TIME ;
//  cur_speed2 = PUSLE2DEG*((double)(Encoder - p_Encoder))/TIME; 
//  err_speed2 = ref_speed2 - cur_speed2 ;
//  
//  cur_rpm2 = cur_speed2/6.0 ;
//  err_rpm2 = err_speed2/6.0 ;
//  sat_rpm2 = (cur_speed2 - ref_speed2)/6;
//  err_val_2 = err_rpm2 ;
//  
//  Op_2 = Kp_2*err_val_2;
//  Oi_2 = Oi_2 + Ki_2*err_val_2 ;//- Kc*(wind_up());//- Kc*(err_angle - cur_angle)  ;
//  pre_Od_2 = Od_2*TIME;
//  Od_2 = N_2*(Kd_2*(err_val_2)-pre_Od_2);
//  
//  out_2 = Op_2+Oi_2+Od_2;
//  
//  return out_2;
//}
void control_gain_init()
{
  {
  //Kp = 0.3;
  //Ki = 0.2;
  //
  //Kd = 0.49451;
  //Kc = 0.5;
  //N = 16.7294;
  
 //Kp = 0.03;
 //Ki = 0.0002;
 //     
 //Kd = 0.008;
 //N = 9.16040789142467;

  //Kp = 0.0341066830716548;
  //Ki = 0.0231130725061613;
  //    
  //Kd = 0.00308559449264262;
  //N = 19.5886289882311;
  
  //Kp = 0.0210739268214095;
  //Ki = 0.0062096248478759;
  //    
  //Kd = 0.016878368643201;
  //N = 42.2857833656479;
  //Kp = 0.895448284276385;        
  //Ki = 0.320422726264529;
  //    
  //Kd = 1;
  //N = 15.4191387639795;
  
  //Kp =0.2749475077148;
  //Ki =0.015167127895989;
  //    
  //Kd =0.125; // oscillating factor
  //Kp_2 =1;//1.5;
  //Ki_2 =0.15;//0.1;
  //    
  //Kd_2 =0.4;//0.7; // oscillating factor
  //N_2 = 9;//7;
  }
  PKp_2 = 8;//2.10869939812182;//0.8;//2.92045203042151;
  PKi_2 = 1.1;//1.28631624955068;//0.0215;// 0.711132437994703;
        
  PKd_2 = 2;//0.150367255370439;//0.76476;//1.35916275795339;
  PN_2 = 0.6;
  
  PKc_2 = (1.0/PKp_2);
  
  SKp_2 = 1.2;
  SKi_2 = 0.02;
  
  SKc_2 = (0.1/SKp_2);
  
  dead_margin = 59;
  
  CKp_2 = 70;
  CKi_2 = 0.05;
  
  CKc_2 = (1/CKp_2);
  
  Kp_3 =1.1;//8.5; 
  Ki_3 =0.09;//0.9;
  Kd_3 =0.3;//5; // oscillating factor  
  N_3 = 5;  
    
  ra = 5.33;
  la = 0.001;
 }
double wind_up(double sat_data)
{
  double PWM = pwm_limit(sat_data);
  double saturation = sat_data-PWM;
  if(saturation>0 && sat_data>0)
  {
    return saturation;
  }
  else if(saturation<0 && sat_data<0)
  {
    return saturation;
  }
  else
  {
    return 0;
  }
}
//double currnt_limit(double cur_in)
//{
//  if(cur_in>=0.0)
//  {
//    if(cur_in>=12
//  }
//}
double I_limit(double I,double P)
{
  double sum = P+I;
  double out = fabs(sum);
  if(sum>=0)
  {
    if(out>=100)
    {
      I = 100.0 - P;
      return I;
    }
    else
    {
      return I;
    }
  }
  else
  {
    if(out>=100)
    {
      I = -(100.0 + P);
      return I;
    }
    else
    {
      return I;
    }
  }
  
}
double position_control_2(double theta, int32_t Encoder, int32_t p_Encoder)
{
  
  ref_encoder2 = (int32_t)(t2/0.12)+30000;
  ref_angle2 = t2;
  cur_angle2 = (double)(PUSLE2DEG*(Encoder-ENCODER_INIT)); 
  err_angle2 = ref_angle2-(cur_angle2); //ENCODER_INIT=30000

  err_val_2 = err_limit2(err_angle2) ;
  
  //cur_speed2 = PUSLE2DEG*((double)(Encoder - p_Encoder))/TIME; 
  //err_speed2 = ref_speed2 - cur_speed2 ;
  
  //cur_rpm2 = cur_speed2/6.0 ;
  //err_rpm2 = err_limit2(err_speed2/6.0);
  //sat_rpm2 = (cur_speed2 - ref_speed2)/6;
  
  
  POp_2 = PKp_2*err_val_2;
  POi_2 = POi_2 + (PKi_2*(err_val_2-(PKc_2*(wind_up(Pout_2))))); //;//- Kc*(wind_up());//- Kc*(err_angle - cur_angle)  ;

  POd_2 = PN_2*((PKd_2*err_val_2)-pre_POd_2);
  pre_POd_2 = POd_2;
  
  Pout_2 = POp_2+POi_2+POd_2;
  
  return Pout_2;
}

double velocity_control_2(double speed, int32_t Encoder, int32_t p_Encoder)
{

  ref_rpm2 = speed/(5.0) ;
  cur_speed2 = ((double)(1/TIME)*PUSLE2DEG)*(double)(Encoder - p_Encoder)/5.0; 
  cur_rpm2 = cur_speed2;
  err_rpm2 = speed - cur_rpm2 ;
  
  //sat_rpm2 = (cur_speed2 - ref_speed2)/6.0;
  
  //err_val_2 = err_rpm2 ;
  err_val_2 = err_rpm2 ;
  
  SOp_2 = SKp_2*err_val_2;
  SOi_2 = I_limit((SOi_2 + (SKi_2*(err_val_2-(SKc_2*(wind_up(Sout_2) ) ) ) )) ,SOp_2 );//- Kc*(wind_up());//- Kc*(err_angle - cur_angle)  ;
  
  //SOd_2 = SN_2*((SKd_2*err_val_2)-pre_SOd_2);
  //pre_SOd_2 = SOd_2*TIME;
  
  Sout_2 = SOp_2+SOi_2;
  
  return Sout_2;
}

double current_control_2(double current,double cur_sensor)
{
  ref_cur2 = 1.75*(current/100.0);
  cur_cur2 = cur_sensor;
  err_cur2 = ref_cur2 - cur_cur2;
  
  COp_2 = CKp_2*err_cur2;
  COi_2 = I_limit(COi_2 + (CKi_2*(err_cur2)-(CKc_2*(wind_up(Cout_2)))),COp_2 ) ;
  
  Cout_2 = COp_2+COi_2;
  test = Cout_2;
}

void pwm_dir3()
{
  if(PWM_out3>=0)
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
    htim1.Instance->CCR2 = (uint16_t)(pwm_limit(PWM_out3));//(uint16_t)(fabs(PWM_out));
    //htim1.Instance->CCR2 = (uint16_t)pwm_limit(PWM_out2);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
    htim1.Instance->CCR2 = (uint16_t)(0.9*(pwm_limit(PWM_out3)));//(uint16_t)(fabs(PWM_out)); //motor pwm default 20
    //htim1.Instance->CCR2 = (uint16_t)pwm_limit(PWM_out2);
  }  
}

double position_control_3(double theta, int32_t Encoder, int32_t p_Encoder)
{
  
  ref_angle3 = t3;
  cur_angle3 = PUSLE2DEG*(Encoder-ENCODER_INIT); 
  err_angle3 = err_limit3(ref_angle3-cur_angle3); //ENCODER_INIT=30000
  
  ref_speed3 = 1.5*(err_angle3)/TIME ; //timcnt 0~150, err_angle_max=100, cnt ratio method
  cur_speed3 = 1.5*(PUSLE2DEG*((double)(Encoder - p_Encoder)))/TIME; 
  err_speed3 = speed_limit(ref_speed3 - cur_speed3) ;
  
  cur_rpm3 = cur_speed3/6;
  err_rpm3 = err_speed3/6 ;
  sat_rpm3 = (cur_angle3 - ref_angle3)/6;
  err_val_3 = err_rpm3 ;
  
  Op_3 = Kp_3*err_val_3;
  Oi_3 = Oi_3 + Ki_3*err_val_3 ;//- Kc*(wind_up());//- Kc*(err_angle - cur_angle)  ;
  pre_Od_3 = Od_3*TIME;
  Od_3 = N_3*(Kd_3*(err_val_3)-pre_Od_3);
  
  out_3 = (Op_3+Oi_3+Od_3);//+(Kc*(sat_rpm3))
  
  return out_3;
}

void motion_activate()
{
  switch(mode)
      {
      case 0:
        {
          //read_encoder();
          //PWM_out2 = position_control_2(theta_2, Encoder3, pre_Encoder3); //////////
          //PWM_out = velcity_control_2(Encoder3, pre_Encoder3);
          //pwm_dir2();
          
          //PWM_out3 = position_control_3(theta_3, Encoder5, pre_Encoder5); //////////
          //pwm_dir3();
          
          endeff_set(theta, theta, theta);
          //cal_target_val(px,py,pz);
          //pos_calibrate();
          
          theta_sum = temp_2 + temp_3 + temp_5 ;
          theta_sum1 = t2 + t3 + t5 ; 
          
          dec2hex(t1, t4, t5, t6) ;
          value_assign();
          Packet_value_set() ; /// xels activate
          //angle_sum = theta_2+theta_3+theta_4 ;
          HAL_UART_Transmit(&huart6, (uint8_t *)bufftx, (sizeof(bufftx)), HAL_MAX_DELAY);
 
          break ;
        }
      case 1:
        {
          Packet_value_set() ; /// xels activate
          //read_encoder();
          HAL_UART_Transmit(&huart6, (uint8_t *)bufftx, (sizeof(bufftx)), HAL_MAX_DELAY);
          // xel default pos
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
          htim1.Instance->CCR1 = 0;
          htim1.Instance->CCR2 = 0;
          
          calibrate();//////////
          
          break ;
        }
      
      }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(mode == MOTOR_OFF)
  {
    calibrate();
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
  else
  {
    if(htim == &htim2)
    {
      for(int i = 0; i < 2; i++)
      {
        temp[i] = adcVal[i];//;-(uint32_t)(3103);
        mot_cur[i] = (5*(double)(temp[i]-2048))/4096; //(3.3/5)=0.66 //4096-3103 = 993
        mot_cur[i] = (filter)*mot_cur[i]+(1.0-filter)*mot_temp[i];
        mot_temp[i] = mot_cur[i]+0.08;
      }
      move_avg[0] = move_avg[1];
      move_avg[1] = move_avg[2];
      move_avg[2] = move_avg[3];
      move_avg[3] = move_avg[4];
      move_avg[4] = move_avg[5];
      move_avg[5] = move_avg[6];
      move_avg[6] = move_avg[7];
      move_avg[7] = move_avg[8];
      move_avg[8] = mot_cur[0];
      
      move_avg[9] = (move_avg[0]+move_avg[1]+move_avg[2]+move_avg[3]+move_avg[4]+move_avg[5]+move_avg[6]+move_avg[7]+move_avg[8])/9.0;
      PWM_out3 = current_control_2(PWM_out2,move_avg[9]);
      pwm_dir2();
    }
    else if(htim == &htim9)
    {
      read_encoder();
      PWM_out2 = pwm_limit2(velocity_control_2(PWM_out, Encoder3, pre_Encoder3));
      
      // motion_activate();
      
    }
    else if(htim == &htim4)// control period 0.01s
    {
      
      read_encoder();
      PWM_out = pwm_limit(position_control_2(theta_2, Encoder3, pre_Encoder3));
      //pwm_dir2();//motion_activate();
      //cur_sum = 100*(mot_cur[0]+mot_cur[1]);
      //move();
      
      //read_encoder();
      
  //      bufftx[8]  = angle_0L;
  //      bufftx[9]  = angle_0H;      
  //      bufftx[10] = speed_0L;
  //      bufftx[11] = speed_0H;
  //    
  //      bufftx[13] = angle_12L;
  //      bufftx[14] = angle_12H;      
  //      bufftx[15] = speed_12L;
  //      bufftx[16] = speed_12H;
  //    
  //      bufftx[18] = angle_12L;
  //      bufftx[19] = angle_12H;      
  //      bufftx[20] = speed_12L;
  //      bufftx[21] = speed_12H;     
  //    
  //      bufftx[23] = angle_3L;
  //      bufftx[24] = angle_3H;      
  //      bufftx[25] = speed_3L;
  //      bufftx[26] = speed_3H; 
  //    
  //      bufftx[28] = angle_4L;
  //      bufftx[29] = angle_4H;      
  //      bufftx[30] = speed_4L;
  //      bufftx[31] = speed_4H;   
  //
  //      bufftx[32] =  ~(0xfe + 0x1d + 0x83 + 0x1e + 0x04 
  //    + 0x00 + angle_0L + angle_0H + speed_0L + speed_0H
  //    + 0x01 + angle_12L + angle_12H + speed_12L + speed_12H
  //    + 0x02 + angle_12L + angle_12H + speed_12L + speed_12H
  //    + 0x03 + angle_3L + angle_3H + speed_3L + speed_3H
  //    + 0x04 + angle_4L + angle_4H + speed_4L + speed_4H );
        
  // {0xff, 0xff, 0x01, 0x05, 0x04, 0x1e, (*angle_L), (*angle_H) , ~( 0x01 + 0x05 + 0x04 + 0x1e + (*angle_L) + (*angle_H))
  //, 0xff, 0xff, 0x02, 0x05, 0x04, 0x1e, 0xff, 0x01, ~( 0x02 + 0x05 + 0x04 + 0x1e + (*angle_L) + (*angle_H))
  //, 0xff, 0xff, 0x00, 0x05, 0x04, 0x1e, 0xff, 0x01, ~( 0x00 + 0x05 + 0x04 + 0x1e + (*angle_L) + (*angle_H))
  //, 0xff, 0xff, 0xfe, 0x02, 0x05, ~(0xfe + 0x02 + 0x05)};
      if(hold == 0)
      {
        
        trajectory();
        endeff_set(theta,theta,theta);
        cal_target_val(px,py,pz);
        pos_calibrate();
        dec2hex(t1, t4, t5, t6) ;
        value_assign();
        Packet_value_set() ; /// xels activate
      }
      //angle_sum = theta_2+theta_3+theta_4 ;
      HAL_UART_Transmit(&huart6, (uint8_t *)bufftx, (sizeof(bufftx)), HAL_MAX_DELAY);
      ///////////////////////
    
      
      range = 5;
      
      
    }
    //else if(htim == &htim3)
    //{
    //  move();
    //}
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  TIM3->CNT = ENCODER_INIT;  //ENCODER_INIT=30000
  HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
  TIM5->CNT = 29864;  //ENCODER_INIT=30000
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  htim1.Instance->CCR1 = 0;
  htim1.Instance->CCR2 = 0;
  
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim9);
  
  HAL_TIM_Base_Start_IT(&htim4); ////adc trigger timer
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim5);
  
  HAL_ADC_Start_DMA(&hadc1, adcVal, 2);//// adc dma start
  
  mode = 1;
  control_gain_init();
  ref_val_init();
  
  endeff_set(theta_ref1, theta_ref2, theta_ref3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 840-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 60000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8400-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
  //Encoder =__HAL_TIM_GET_COUNTER(&htim2);
  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
