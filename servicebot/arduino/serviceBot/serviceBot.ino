
/*
#  Chefbot_ROS_Interface.ino
# 
#  Adapted for an Arduino. Specifically, the Arduino Duemilanove by 
#  Peter Chau.
#
#  Copyright 2015 Lentin Joseph <qboticslabs@gmail.com>
#  Website : www.qboticslabs.com , www.lentinjoseph.com
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  Some of the portion is adapted from I2C lib example code for MPU 6050
*/

#include "TFMini.h"
TFMini tfmini;
//MPU 9250 Interfacing libraries
#include "MPU9250.h"
//Processing incoming serial data 
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>

//Creating MPU6050 Object
MPU9250 IMU(Wire,0x68);
//Messenger object
Messenger Messenger_Handler = Messenger();

int status;
///////////////////////////////////////////////////////////////
//Encoder pins definition

// Left encoder

#define Left_Encoder_PinA 2
#define Left_Encoder_PinB 12

volatile long Left_Encoder_Ticks = 0;
volatile bool LeftEncoderBSet;

//Right Encoder

#define Right_Encoder_PinA 3
#define Right_Encoder_PinB 11
volatile long Right_Encoder_Ticks = 0;
volatile bool RightEncoderBSet;

/////////////////////////////////////////////////////////////////
//Motor Pin definition
//Left Motor pins

#define A_1 8
#define B_1 9

//PWM 1 pin number
#define PWM_1 10


//Right Motor
#define A_2 7
#define B_2 6

//PWM 2 pin number
#define PWM_2 5

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Ultrasonic pins definition

long duration, cm;

/////////////////////////////////////////////////////////////////////////////////////////
//Time  update variables

unsigned long LastUpdateMicrosecs = 0;		
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

///////////////////////////////////////////////////////////////////////////////////////
//Motor speed from PC
//Motor left and right speed
float motor_left_speed = 0;
float motor_right_speed = 0;
/////////////////////////////////////////////////////////////////


//Setup serial, encoders, ultrasonic, MPU6050 and Reset functions
void setup()
{
  
  //Init Serial port with 115200 baud rate
  Serial.begin(115200);  
  
  //Setup Encoders
  SetupEncoders();
  //Setup Motors
  SetupMotors();
  //Setup Ultrasonic
  SetupUltrasonic();  
  //Setup MPU 6050
  Setup_MPU9250();
  //Setup Reset pins
//  SetupReset();
  //Set up Messenger 
  Messenger_Handler.attach(OnMssageCompleted);
    
  
  
}

//SetupEncoders() Definition

void SetupEncoders()
{
  // Quadrature encoders
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT);      // sets pin A as input  
  pinMode(Left_Encoder_PinB, INPUT);      // sets pin B as input
  //Attaching interrupt in Left_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(Left_Encoder_PinA), do_Left_Encoder, RISING);
  

  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT);      // sets pin A as input
  pinMode(Right_Encoder_PinB, INPUT);      // sets pin B as input
  //Attaching interrupt in Right_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, RISING); 

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup Motors() function

void SetupMotors()
{
 
 //Left motor
 pinMode(A_1,OUTPUT);
 pinMode(B_1,OUTPUT); 
 

 //Right Motor
 pinMode(A_2,OUTPUT);
 pinMode(B_2,OUTPUT);  
  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup UltrasonicsSensor() function
void SetupUltrasonic()
{
  Serial1.begin(TFMINI_BAUDRATE);
  tfmini.begin(&Serial1);    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup MPU6050 function

void Setup_MPU9250()
{
status = IMU.begin();
 
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP

void loop()
{

    //Read from Serial port
    Read_From_Serial();
    
    
    //Send time information through serial port
    Update_Time();
    
    //Send encoders values through serial port
    Update_Encoders();
    
    //Send ultrasonic values through serial port
    Update_Ultra_Sonic();
        

    //Update motor values with corresponding speed and send speed values through serial port
    Update_Motors();


    //Send MPU 6050 values through serial port
    Update_MPU9250();
    
    //Send battery values through serial port
//    Update_Battery();
    
    
  
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function

void Read_From_Serial()

{
   while(Serial.available() > 0)
    {
     
       int data = Serial.read();
       Messenger_Handler.process(data);
     
     
    } 
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition

void OnMssageCompleted()
{
   
  char reset[] = "r";
  char set_speed[] = "s";
  
  if(Messenger_Handler.checkString(reset))
  {
    
     Serial.println("Reset Done"); 
//     Reset();
    
  }
  if(Messenger_Handler.checkString(set_speed))
  {
    
     //This will set the speed
     Set_Speed();
     return; 
    
    
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//do_Left_Encoder() Definitions
void do_Left_Encoder()
{
   // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  Left_Encoder_Ticks += LeftEncoderBSet ? -1 : +1;
   
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//do_Right_Encoder() Definitions

void do_Right_Encoder()
{
  
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks -= RightEncoderBSet ? -1 : +1;
 
 
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Set speed
void Set_Speed()
{
    
  motor_left_speed = Messenger_Handler.readLong();
  motor_right_speed = Messenger_Handler.readLong();
  
  
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update both motors
void Update_Motors()
{
  
  moveRightMotor(motor_right_speed);
  moveLeftMotor(motor_left_speed);

  Serial.print("s");
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\t");
  Serial.print(motor_right_speed);  
  Serial.print("\n");


}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update both encoder value through serial port
void Update_Encoders()
{
 
  Serial.print("e");
  Serial.print("\t");
  Serial.print(Left_Encoder_Ticks);
  Serial.print("\t");
  Serial.print(Right_Encoder_Ticks);
  Serial.print("\n");
  
  
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update ultrasonic sensors through serial port

void Update_Ultra_Sonic()
{
  uint16_t cm = tfmini.getDistance();
  
  //Sending through serial port
  Serial.print("u");
  Serial.print("\t");
  Serial.print(cm);
  Serial.print("\n");
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update MPU6050

void Update_MPU9250()
{
  

    IMU.readSensor();
    
    Serial.print("i");Serial.print("\t");
    Serial.print(IMU.getAccelX_mss(),4); Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(),4); Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(),4); Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(),4); Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(),4); Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(),4);
    Serial.print("\n");
 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update time function
void Update_Time()
{
  
      
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0)
    {
	MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;

    }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;

  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
 
  
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update battery function
//void Update_Battery()
//
//{
// battery_level = analogRead(PC_4); 
// 
// Serial.print("b");
// Serial.print("\t");
// Serial.print(battery_level);
// Serial.print("\n");
//
//}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Motor running function


void moveRightMotor(float rightServoValue)
{
  if (rightServoValue>0)
  {
       
 digitalWrite(A_1,HIGH);
 digitalWrite(B_1,LOW);
 analogWrite(PWM_1,rightServoValue);
    
  }
  else if(rightServoValue<0)
  {
 digitalWrite(A_1,LOW);
 digitalWrite(B_1,HIGH);
 analogWrite(PWM_1,abs(rightServoValue));
 
  }
  
  else if(rightServoValue == 0)
  {
 digitalWrite(A_1,HIGH);
 digitalWrite(B_1,HIGH);
    
    
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveLeftMotor(float leftServoValue)
{
 if (leftServoValue > 0)
  {
digitalWrite(A_2,LOW);
digitalWrite(B_2,HIGH);
analogWrite(PWM_2,leftServoValue);
  }
  else if(leftServoValue < 0)
  {
 digitalWrite(A_2,HIGH);
 digitalWrite(B_2,LOW);
 analogWrite(PWM_2,abs(leftServoValue));

  }
  else if(leftServoValue == 0)
  {

   digitalWrite(A_2,HIGH);
   digitalWrite(B_2,HIGH);
  
   }  
  
  
}

