/************************   Chiangmai Maker Club   *****************************
 * Project  : Levitation System with Arduino                                   *
 * Compiler : Arduino 1.5.5-r2                                                 *
 * Board    : Arduino UNO                                                      *
 * Device   : Spire + Steel rods                                               *
 *          : Hall Effecr Sensor A1302                                         *
 *          : FET. IRF540 with diode 1N4007                                    *
 *          : Splat or Hinoki wood                                             *
 * Modified : wasin wongkum                                                    *
 * Copy     : Bavensky :3                                                      *
 * E-Mail   : Apiruk_Sang-ngenchai@hotmail.com                                 *
 * Date     : 06/12/2557 [dd/mm/yyyy]                                          *
 * Read more: http://en.wikipedia.org/wiki/Low-pass_filter <-- Low-pass filter *
 *          : http://en.wikipedia.org/wiki/Hall_effect_sensor <-- Hall Sensor  *
 *******************************************************************************/
 
#include <TimerOne.h>              // เรียกใช้ไลบรารี่ Timer1 

#define hall        A3             // กำหนดต่อ Hall effect Sensor ที่ขา Analog 0
#define magnetic     5            // กำหนดขาเอาต์พุตของขดลวดที่จะทำให้เป็นแม่เหล็ก เป็นขา PWM ของบอร์ด Arduino
#define filter      0.4f           // กำหนดค่าเพื่อใช้ในสูตร Low-pass filter = for i from 1 to n เมื่อ y[i] := y[i-1] + α * (x[i] - y[i-1])       
#define sampling    1000.0f        // กำหนดให้รอบการทำงานที่ 1 Khz

float state = 0;                   // ตำแหน่งปัจจุบัน
float prev_state = 0;              // ตำแหน่งก่อนหน้านี้  
float ref = 6.7;                   // ตำแหน่งที่ต้องการให้ลอยอยู่

float error = 0;                   // ใช้สำหรับเก็บค่าส่วนต่างจากการเคลื่อนที่ของวัตถุปัจจุบัน
float prev_error =0;               // ใช้สำหรับเก็บค่าส่วนต่างจากการเคลื่อนที่ของวัตถุก่อนหน้านี้
float error_dot = 0;               // ใช้สำหรับเก็บค่าที่ผิดพลาด
float error_sum =0;                // ใช้สำหรับเก็บค่าผมรวมที่ผิดพลาดในการทำงาน

float kp = 400;                    // ค่า P (สปริง)
float ki = 8;                     // ค่า I (ชดเชย)
float kd = 3;                    // ค่า D (โช๊คอัพ)

float output  = 0;                 // ค่าที่จะส่งออกไปให้ FET ทำงาน (เป็น PWM)

void setup() 
{
  Serial.begin(115200);
  pinMode(hall, INPUT);            // ประกาศค่าให้ hall เป็นอินพุต
  pinMode(output, OUTPUT);         // ประกาศค่าให้ output เป็นเอาต์พุต
  
  Timer1.initialize(1000);         // อินเตอร์รัพท์ให้ทำงานที่ 1 Khz
  Timer1.attachInterrupt( PID );   // ใน 1Khz ของการอินเตอร์รัพท์มห้ทำงาน PID(void) ไปด้วย
}

void loop() 
{
//    delay(1000);                   // หน่วงเวลา 1 วินาที เพื่อให้สามารถดูค่าต่าง ๆ ได้ง่าย
//    Serial.print("state =");       // แสดงค่า ตำแหน่งปัจจุบันที่อ่านได้จาก Hall Effect Sensor
//    Serial.print(state);
//    Serial.print(" error=");       // แสดงค่าที่ผิดพลาดจากตำแหน่งปัจจุบันกับตำแหน่งก่อนหน้านี้
//    Serial.print(error);
//    Serial.print(" error_dot=");   // แสดงค่าที่ผิดพลาดเพื่อใช้สำหรับการคืนค่าให้อยู่ในตำแหน่งที่ต้องการ
//    Serial.print(error_dot);
//    Serial.print(" error_sum=");   // แสดงค่าที่ผิดพลาดสะสมที่เกิดจากการเออเรอร์ 
//    Serial.print(error_sum);
//    Serial.print(" output=");      // แสดงค่า PWM ที่ส่งออกไปควบคุม FET
//    Serial.println(output);
}

void PID(void) 
{  

  prev_state = state ;             // บันทึกค่าตำแหน่งปัจจุบันไว้ในตัวแปรตำแหน่งก่อนหน้านี้
  
  state = prev_state + (filter*(((float)(analogRead(hall))/100.0f)-prev_state));    // ใช้สมการ Low pass filter ในการอ่านค่าจาก Hall Effect Sensor เพื่อเก็บค่าไว้ในตำแหน่งปัจจุบัน
    
  prev_error = error;              // เก็บค่าผิดพลากก่อนหน้านี้
  error = ref - state;             // ตำแหน่งที่ต้องการ ลบกับ ตำแหน่งที่อ่านค่าได้ เก็บไว้ในตัวแปร error เช่นถ้า ref = 7 และ state = 5 ค่าผิดพลาดจะได้ท่ากับ 2 
  error_dot = (error - prev_error)*sampling;    // นำค่าผิดพลาด ลบกับ ค่าผิดพลากก่อนหน้านี้ แล้วคูณด้วยรอบการทำงาน เพื่อนำไปคูณกับค่า kd
  
  error_sum = error_sum + error/sampling;       // นำค่าผิดพลาดในแต่ละครั้งหารด้วยรอบการให้งาน 1Khz = 1000 hz หารด้วยค่าผิดพลาก ค่าที่ได้จะเป็นค่าหลักหน่วย 
 
  output = kp*error + ki*error_sum + kd*error_dot;  // นำ kp, ki, kd มารวมกันแล้วส่งค่าออกทางเอาต์พุตไปที่ FET
  
  /*****  ฟังก์ชั่นสำหรับควบคุมเอาต์พุต(PWM) ให้ออกในค่า 0 - 255  *****/
  if(output<0)   output = 0;
  if(output>255) output = 255;
  
  /*****  ฟังก์ชั่นสำหรับควบคุมการทำงานของโปรแกรม ถ้าไม่เจอ Sensor จะไม่ส่งค่าไปยังเอาต์พุต FET จะไม่ทำงาน แล้วขดลวดจะไม่เกิดสนามแม่เหล็ก  *****/
  if(state < 5.8)
  {
    output = 0;
    error_sum=0;
  }
 
  analogWrite(magnetic,  output);    // ส่งค่า output(PWM) ออกไปยัง magnetic หรือขา G ของ FET
}
