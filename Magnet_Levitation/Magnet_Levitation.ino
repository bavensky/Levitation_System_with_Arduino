/************************   Chiangmai Maker Club   *****************************
 * Project  : Levitation System with Arduino                                   *
 * Compiler : Arduino 1.5.5-r2                                                 *
 * Board    : Arduino UNO                                                      *
 * Device   : Spire + Steel rods                                               *
 *          : Hall Effecr Sensor A1302                                         *
 *          : FET. IRF540 with diode 1N4007                                    *
 * Modified : wasin wongkum                                                    *
 * Copy     : Bavensky :3                                                      *
 * E-Mail   : Aphirak_Sang-ngenchai@hotmail.com                                *
 * Date     : 06/12/2557 [dd/mm/yyyy]                                          *
 *******************************************************************************/
 
#include <TimerOne.h>

#define hall        A3     // กำหนดต่อ Hall effect Sensor ที่ขา Analog 0
#define magnetic     11     // 
#define filter      0.4f   
#define sampling    1000.0f 

float state_g = 0;
float prev_state_g = 0;

float state = 0;
float prev_state = 0;

float ref = 6.6;
float error = 0;
float prev_error =0;
float error_dot = 0;
float error_sum =0;

float kp = 515;
float ki = 10;
float kd = 3.35;

float output  = 0;

void setup() 
{
  Serial.begin(115200);
  pinMode(hall, INPUT);
  pinMode(A1, INPUT);
  pinMode(output, OUTPUT);
  pinMode(12, OUTPUT); 
  digitalWrite(12, 1);
  
  Timer1.initialize(1000); // sampling rate 1 kHz
  Timer1.attachInterrupt( PID );
}

void loop() 
{
    //PID();
    delay(1000);
//    Serial.print(analogRead(A1));
//    Serial.print("     ");
    Serial.print("state =");
    Serial.print(state);
    Serial.print("     ki=");
    Serial.print(error_dot);
    Serial.print("     error=");
    Serial.print(error);
    Serial.print("     error_sum=");     
    Serial.print(error_sum);
    Serial.print("     output="); 
    Serial.println(output);
}

void PID(void) 
{  
  //digitalWrite(12,1);
  
//  prev_state_g = state_g ;
//  state_g = prev_state_g + (filter*(((float)(analogRead(A1))/100.0f)-prev_state_g));
//  kd = state_g /1.0f ;
//  
//  prev_error = error;
//  error = ref - state_g;
//  error_dot = (error - prev_error)*sampling;
//  error_sum = error_sum + error/sampling;

  prev_state = state ;
  state = prev_state + (filter*(((float)(analogRead(hall))/100.0f)-prev_state));    // Low pass filter
  
  //if(state < 5.6) state = 5.5;
    
  prev_error = error;      //  
  error = ref - state;
  error_dot = (error - prev_error)*sampling;
  error_sum = error_sum + error/sampling;
 
  output = kp*error + ki*error_sum + kd*error_dot;
  
  if(output<0)   output = 0;
  if(output>255) output = 255;
  
  if(state < 5.8)
  {
    output = 0;
    error_sum=0;
  }
  
  
  analogWrite(magnetic,output);
  //digitalWrite(12,0);
}
