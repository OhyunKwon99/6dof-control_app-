#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "6dof-xel.h"
#include "stm32f4xx_hal.h"

extern const double half_value;

extern uint16_t low ;               // round function parameter 
extern double p_point, m_point ;    // round function parameter
extern uint8_t mid;
extern uint8_t H, L ;

///////////////////////////////dynamixel hex data
extern double hex_1, hex_2, hex_3, hex_4 ;          // dynamixel hex data 
 
extern unsigned char 
 hex_1_L, hex_1_H,                // dynamixel data 
 hex_2_L, hex_2_H,                             // dynamixel data 
 hex_3_L, hex_3_H,                             // dynamixel data 
 hex_4_L, hex_4_H ;                            // dynamixel data 


void round1()    // round off func
{
  low = (int)(hex_1);
  mid = (int)((hex_1-(double)(low))/half_value);
  H = (uint8_t)(hex_1/256);
  L = (uint8_t)(hex_1-256*H);
  switch (mid)
  {
  case 0:
    break;
    
  case 1:
    if(L != 255)
    {
      L = L + 1;
    }
    else
    {
      H = H + 1;
      L = 0;
    }
    break ;
    
  case 2:
    if(L != 255)
    {
      L = L + 1;
    }
    else
    {
      H = H + 1;
      L = 0;
    }
    break ;
    
  case 3:
    if((L != 255)&&(L != 254)) 
    { 
     L = L + 2; 
    } 
    else if(L == 255)
    { 
      H = H + 1; 
      L = 1; 
    }
    else
    {
      H = H + 1; 
      L = 0;
    }
    break ; 
    
  case 4:
    if((L != 255)&&(L != 254)) 
    { 
     L = L + 2; 
    } 
    else if(L == 255)
    { 
      H = H + 1; 
      L = 1; 
    }
    else
    {
      H = H + 1; 
      L = 0;
    }
    break ;  
    
  case 5:        
    if((L != 255)&&(L != 254)&&(L != 253))      
    {      
      L = L + 3;      
    }      
    else if(L == 255)
    { 
      H = H + 1;
      L = 2; 
    }
    else if(L == 254)
    { 
      H = H + 1; 
      L = 1; 
    }
    else 
    {
      H = H + 1;
      L = 0;
    }
    break ; 
         
  case 6:        
    if((L != 255)&&(L != 254)&&(L != 253))      
    {      
      L = L + 3;      
    }      
    else if(L == 255)
    { 
      H = H + 1;
      L = 2; 
    }
    else if(L == 254)
    { 
      H = H + 1; 
      L = 1; 
    }
    else 
    {
      H = H + 1;
      L = 0;
    }
    break ;      
    
  }
  hex_1_H = (H);
  hex_1_L = (L);
  
}
void round2()    // round off func
{
  low = (int)(hex_2);
  mid = (int)((hex_2-(double)(low))/half_value);
  H = (uint8_t)(hex_2/256);
  L = (uint8_t)(hex_2-256*H);
  switch (mid)
  {
  case 0:
    break;
    
  case 1:
    if(L != 255)
    {
      L = L + 1;
    }
    else
    {
      H = H + 1;
      L = 0;
    }
    break ;
    
  case 2:
    if(L != 255)
    {
      L = L + 1;
    }
    else
    {
      H = H + 1;
      L = 0;
    }
    break ;
    
  case 3:
    if((L != 255)&&(L != 254)) 
    { 
     L = L + 2; 
    } 
    else if(L == 255)
    { 
      H = H + 1; 
      L = 1; 
    }
    else
    {
      H = H + 1; 
      L = 0;
    }
    break ; 
    
  case 4:
    if((L != 255)&&(L != 254)) 
    { 
     L = L + 2; 
    } 
    else if(L == 255)
    { 
      H = H + 1; 
      L = 1; 
    }
    else
    {
      H = H + 1; 
      L = 0;
    }
    break ;  
    
  case 5:        
    if((L != 255)&&(L != 254)&&(L != 253))      
    {      
      L = L + 3;      
    }      
    else if(L == 255)
    { 
      H = H + 1;
      L = 2; 
    }
    else if(L == 254)
    { 
      H = H + 1; 
      L = 1; 
    }
    else 
    {
      H = H + 1;
      L = 0;
    }
    break ; 
         
  case 6:        
    if((L != 255)&&(L != 254)&&(L != 253))      
    {      
      L = L + 3;      
    }      
    else if(L == 255)
    { 
      H = H + 1;
      L = 2; 
    }
    else if(L == 254)
    { 
      H = H + 1; 
      L = 1; 
    }
    else 
    {
      H = H + 1;
      L = 0;
    }
    break ;      
    
  }
  hex_2_H = (H);
  hex_2_L = (L);
}
void round3()    // round off func
{
  low = (int)(hex_3);
  mid = (int)((hex_3-(double)(low))/half_value);
  H = (uint8_t)(hex_3/256);
  L = (uint8_t)(hex_3-256*H);
  switch (mid)
  {
  case 0:
    break;
    
  case 1:
    if(L != 255)
    {
      L = L + 1;
    }
    else
    {
      H = H + 1;
      L = 0;
    }
    break ;
    
  case 2:
    if(L != 255)
    {
      L = L + 1;
    }
    else
    {
      H = H + 1;
      L = 0;
    }
    break ;
    
  case 3:
    if((L != 255)&&(L != 254)) 
    { 
     L = L + 2; 
    } 
    else if(L == 255)
    { 
      H = H + 1; 
      L = 1; 
    }
    else
    {
      H = H + 1; 
      L = 0;
    }
    break ; 
    
  case 4:
    if((L != 255)&&(L != 254)) 
    { 
     L = L + 2; 
    } 
    else if(L == 255)
    { 
      H = H + 1; 
      L = 1; 
    }
    else
    {
      H = H + 1; 
      L = 0;
    }
    break ;  
    
  case 5:        
    if((L != 255)&&(L != 254)&&(L != 253))      
    {      
      L = L + 3;      
    }      
    else if(L == 255)
    { 
      H = H + 1;
      L = 2; 
    }
    else if(L == 254)
    { 
      H = H + 1; 
      L = 1; 
    }
    else 
    {
      H = H + 1;
      L = 0;
    }
    break ; 
         
  case 6:        
    if((L != 255)&&(L != 254)&&(L != 253))      
    {      
      L = L + 3;      
    }      
    else if(L == 255)
    { 
      H = H + 1;
      L = 2; 
    }
    else if(L == 254)
    { 
      H = H + 1; 
      L = 1; 
    }
    else 
    {
      H = H + 1;
      L = 0;
    }
    break ;      
    
  }
  hex_3_H = (H);
  hex_3_L = (L);
}
void round4()    // round off func
{
  low = (int)(hex_4);
  mid = (int)((hex_4-(double)(low))/half_value);
  H = (uint8_t)(hex_4/256);
  L = (uint8_t)(hex_4-256*H);
  switch (mid)
  {
  case 0:
    break;
    
  case 1:
    if(L != 255)
    {
      L = L + 1;
    }
    else
    {
      H = H + 1;
      L = 0;
    }
    break ;
    
  case 2:
    if(L != 255)
    {
      L = L + 1;
    }
    else
    {
      H = H + 1;
      L = 0;
    }
    break ;
    
  case 3:
    if((L != 255)&&(L != 254)) 
    { 
     L = L + 2; 
    } 
    else if(L == 255)
    { 
      H = H + 1; 
      L = 1; 
    }
    else
    {
      H = H + 1; 
      L = 0;
    }
    break ; 
    
  case 4:
    if((L != 255)&&(L != 254)) 
    { 
     L = L + 2; 
    } 
    else if(L == 255)
    { 
      H = H + 1; 
      L = 1; 
    }
    else
    {
      H = H + 1; 
      L = 0;
    }
    break ;  
    
  case 5:        
    if((L != 255)&&(L != 254)&&(L != 253))      
    {      
      L = L + 3;      
    }      
    else if(L == 255)
    { 
      H = H + 1;
      L = 2; 
    }
    else if(L == 254)
    { 
      H = H + 1; 
      L = 1; 
    }
    else 
    {
      H = H + 1;
      L = 0;
    }
    break ; 
         
  case 6:        
    if((L != 255)&&(L != 254)&&(L != 253))      
    {      
      L = L + 3;      
    }      
    else if(L == 255)
    { 
      H = H + 1;
      L = 2; 
    }
    else if(L == 254)
    { 
      H = H + 1; 
      L = 1; 
    }
    else 
    {
      H = H + 1;
      L = 0;
    }
    break ;      
    
  }
  hex_4_H = (H);
  hex_4_L = (L);
}
