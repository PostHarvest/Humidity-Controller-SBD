#define SPI_MODE_0  (SPI_L_TO_H | SPI_XMIT_L_TO_H) 
#define SPI_MODE_1  (SPI_L_TO_H) 
#define SPI_MODE_2  (SPI_H_TO_L) 
#define SPI_MODE_3  (SPI_H_TO_L | SPI_XMIT_L_TO_H) 
#define CS PIN_B6
//#use spi(DI=PIN_B0, DO = PIN_B7, CLK=PIN_B1) 
#use spi(SPI1, MASTER, baud=100000, MODE=0, bits=8, stream=MAX31855) 
//#use SPI( DO = PIN_C0, DI = PIN_C4, CLK = PIN_C3, baud = 100000, BITS = 8, LSB_FIRST, SAMPLE_RISE ) 
const int16 NegativeK[150]={  0,  39,  79, 118, 157, 197, 236, 275, 314, 353, 
                      392, 431, 470, 508, 547, 586, 624, 663, 701, 739, 
                      778, 816, 854, 892, 930, 968,1006,1043,1081,1119, 
                     1156,1194,1231,1268,1305,1343,1380,1417,1453,1490, 
                     1527,1564,1600,1637,1673,1709,1745,1782,1818,1854, 
                     1889,1925,1961,1996,2032,2067,2103,2138,2173,2208, 
                     2243,2278,2312,2347,2382,2416,2450,2485,2519,2553, 
                     2587,2620,2654,2688,2721,2755,2788,2821,2854,2887, 
                     2920,2953,2986,3018,3050,3083,3115,3147,3179,3211, 
                     3243,3274,3306,3337,3368,3400,3431,3462,3492,3523, 
                     3554,3584,3614,3645,3675,3705,3734,3764,3794,3823, 
                     3852,3882,3911,3939,3968,3997,4025,4054,4082,4110, 
                     4138,4166,4194,4221,4249,4276,4303,4330,4357,4384, 
                     4411,4437,4463,4490,4516,4542,4567,4593,4618,4644, 
                     4669,4694,4719,4744,4768,4793,4817,4841,4865,4889}; 

const int16 PositiveK[150]={  0,  39,  79, 119, 158, 198, 238, 277, 317, 357, 
                      397, 437, 477, 517, 557, 597, 637, 677, 718, 758, 
                      798, 838, 879, 919, 960,1000,1041,1081,1122,1163, 
                     1203,1244,1285,1326,1366,1407,1448,1489,1530,1571, 
                     1612,1653,1694,1735,1776,1817,1858,1899,1941,1982, 
                     2023,2064,2106,2147,2188,2230,2271,2312,2354,2395, 
                     2436,2478,2519,2561,2602,2644,2685,2727,2768,2810, 
                     2851,2893,2934,2976,3017,3059,3100,3142,3184,3225, 
                     3267,3308,3350,3391,3433,3474,3516,3557,3599,3640, 
                     3682,3723,3765,3806,3848,3889,3931,3972,4013,4055, 
                     4096,4138,4179,4220,4262,4303,4344,4385,4427,4468, 
                     4509,4550,4591,4633,4674,4715,4756,4797,4838,4879, 
                     4920,4961,5002,5043,5084,5124,5165,5206,5247,5288, 
                     5328,5369,5410,5450,5491,5532,5572,5613,5653,5694, 
                     5735,5775,5815,5856,5896,5937,5977,6017,6058,6098}; 


static int8 spival[4]; 
int16 Count=0; 
/******************************************************************************* 
//Read SPI data 
*******************************************************************************/  
/*
void Read4Byte()
{
//spi_xfer();

spival[3] = spi_xfer(SPI_1,0);
spival[2] = spi_xfer(SPI_1,0);
spival[1] = spi_xfer(SPI_1,0);
spival[0] = spi_xfer(SPI_1,0);

}
*/
void readMAX() 
{ 
   spi_xfer(MAX31855,0); //switches the stream to mode 2     
   output_low(CS); 
   delay_us(10);
   spival[3]=spi_xfer(MAX31855,0); 
   delay_us(10);
   spival[2]=spi_xfer(MAX31855,0);
    delay_us(10);
   spival[1]=spi_xfer(MAX31855,0);
    delay_us(10);
   spival[0]=spi_xfer(MAX31855,0); 
  delay_us(10);
   output_high(CS); 
} 
/*
void readMAX() 
{ 
      output_low(CS); 
      delay_us(1); 
      spival[3]=spi_read(0); 
     // delay_us(10);
      spival[2]=spi_read(0);
      //delay_us(10);
      spival[1]=spi_read(0);
     // delay_us(10);
      spival[0]=spi_read(0); 
      delay_us(1); 
      output_high(CS); 
} 
*/
/******************************************************************************* 
//Fault detection. 
//Returns >0 if FAULT. If necessary do a bitwise analisys to check fault source 
*******************************************************************************/ 
int tempFault() 
{ 
      int1 Fvdd=0,Fgnd=0,Fopen=0,fault=0; 
  
      fault=spival[2]&0x01; // pelos menos uma falha 
      Fvdd=(spival[0]>>2)&0x01; 
      Fgnd=(spival[0]>>1)&0x01; 
      Fopen=spival[0]&0x01; 
      
      return (fault*1+Fvdd*2,Fgnd*4,Fopen*8); 
} 

/******************************************************************************* 
//Read thermocouple temperature 
//Returns returns signed temperature in ºC approximately 
*******************************************************************************/ 
float readExtTemp() 
{ 
      int16 temp1=0; 
      float aux=0; 
  
//spival[3]=0x01; 
//spival[2]=0x90; 
//Count++; 

      temp1=make16(spival[3],spival[2]); 
//temp1-=Count; 

//fprintf(lcd_putc,"REXT:%LX , ",Temp1); 
    
      if(!bit_test(temp1,15)) 
      { 
         temp1>>=2; 
//fprintf(lcd_putc,"SR+EXT:%LX , ",Temp1); 
         aux=(temp1/4.00); 
//fprintf(lcd_putc,"Ext+C:%3.2f, ",aux); 
            return aux; 
      } 
      else 
      { 
         temp1=~temp1; 
         temp1>>=2; 
         temp1+=1; 
//fprintf(lcd_putc,"SR-EXT:%LX , ",Temp1); 
         aux=((temp1/4.00)*-1.00); 

//fprintf(lcd_putc,"RAW Ext-C:%3.2f, ",aux); 
            return aux; 
      } 
} 

/******************************************************************************* 
//Read internal temperature 
//Returns returns signed temperature in ºC approximately 
*******************************************************************************/ 
float readIntTemp() 
{ 
    int16 temp2=0; 
   float aux=0; 
      
   temp2=make16(spival[1],spival[0]); 
//fprintf(lcd_putc,"RINT:%LX , ",Temp2); 

   if(!bit_test(temp2,15)) 
   { 
      temp2>>=4; 
//fprintf(lcd_putc,"SR+INT:%LX , ",Temp2); 
      aux=(temp2/16.00); 
//fprintf(lcd_putc,"Int+C:%3.2f\r\n",aux); 
         return aux; 
    }  
   else 
   { 
      temp2=~temp2; 
      temp2>>=4; 
      temp2+=1; 
//fprintf(lcd_putc,"SR-INT:%LX , ",Temp2); 
      aux=((temp2/16.00)*-1.00); 
//fprintf(lcd_putc,"RAW Int-C:%3.2f\r\n",aux); 
         return aux; 
   }  
} 

float PreFilter() 
{ 
float Raw_TC_Temp[3]={0.00,0.00,0.00}; 
int Raw_Temp_Index=0; 
int1 Temp_Stable=0; 
       
   while(Temp_Stable==0) 
   { 
      Raw_TC_Temp[0]=0.00; 
      Raw_TC_Temp[1]=0.00; 
      Raw_TC_Temp[2]=0.00; 
      Raw_Temp_Index=0; 

      while(Raw_Temp_Index<3) 
      { 
       do{readMAX();}while(tempFault()!=0); 
      Raw_TC_Temp[Raw_Temp_Index]=readExtTemp(); 
//Raw_TC_Temp[Raw_Temp_Index]=-70.00;               // DEBUG 
      Raw_Temp_Index++; 
      delay_ms(100); 
      } 
      if((abs(Raw_TC_Temp[0]-Raw_TC_Temp[1])<1.25)&&(abs(Raw_TC_Temp[1]-Raw_TC_Temp[2])<1.25)&&(abs(Raw_TC_Temp[0]-Raw_TC_Temp[2])<1.25)) 
      Temp_Stable=1; 
   } 
//fprintf(lcd_putc,"RAW1:%3.2f - RAW2:%3.2f - RAW3:%3.2f \r\n",Raw_TC_Temp[0],Raw_TC_Temp[1],Raw_TC_Temp[2]); 
   return((Raw_TC_Temp[0]+Raw_TC_Temp[1]+Raw_TC_Temp[2])/3.00); 
} 

float Linearized_TC() 
{ 
   const float Curve_Value=0.041276; 
   float EXT_Minus_INT=0.00; 
   float TC_Voltage=0.00; 
   float INT_Temp=0.00; 
   float EXT_Temp=0.00; 
   int NIST_Index=0; 

    
   EXT_Temp=PreFilter(); 
   INT_Temp=readIntTemp(); 

   if(EXT_Temp<0.00) 
   { 
      EXT_Minus_INT=(EXT_Temp-INT_Temp); 
      TC_Voltage=EXT_Minus_INT*Curve_Value; 
//fprintf(lcd_putc,"TC_Voltage:%3.2f\r\n",TC_Voltage);   //MATH OK 

//fprintf(lcd_putc,"DEC_INT:%3.2f\r\n",INT_Temp);    
      while(NIST_Index<INT_Temp)NIST_Index++; 
      NIST_Index--; 

//fprintf(lcd_putc,"NIST:%d\r\n",NIST_Index);    
//fprintf(lcd_putc,"NIST MV:%ld\r\n",PositiveK[NIST_Index]);    
      TC_Voltage+=(PositiveK[NIST_Index]/1000.00); 
      TC_Voltage*=-1000.00;    
//fprintf(lcd_putc,"TC_Voltage2:%3.2f\r\n",TC_Voltage); 
    
      NIST_Index=0; 
      while(TC_Voltage>NegativeK[NIST_Index]) 
      { 
         NIST_Index++; 
      } 
      NIST_Index--; 
//fprintf(lcd_putc,"EXT NIST:%d\r\n",NIST_Index);    
      return(NIST_Index*-1.00); 
   } 
   else 
      return(EXT_Temp); 
} 
