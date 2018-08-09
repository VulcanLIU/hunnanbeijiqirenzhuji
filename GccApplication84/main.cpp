/*
  main.cpp - Main loop for Arduino sketches
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#define F_CPU 16000000UL
#include "Arduino.h"
#include "Timer1.h"
#include "PID_v1.h"

//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=2, aggKi=0, aggKd=0;
double consKp=1, consKi=0.05, consKd=0.25;

String kp="",ki="",kd="";

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//定时器
Timer1 tc1;
void report_encode();
void increase();

//全局变量区
int MAX_encode_num = 0;
int encode_num = 0;
int ADC_val = 0;

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

void setupUSB() __attribute__((weak));
void setupUSB() { }


int main(void)
{
	init();

	initVariant();
	
#if defined(USBCON)
	USBDevice.attach();
#endif
	
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
        
	return 0;
}


#define VRX A5
#define VRY A4
#define interruptPin 2
#define motor_A 5
#define motor_B 6

void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);
	//设置中断
	pinMode(interruptPin,INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(interruptPin), increase, FALLING);
	//设置定时器
	tc1.setMode("CTC",50);
	tc1.attachInterrupt(report_encode);
	//驱动电机
	pinMode(motor_A,OUTPUT);
	analogWrite(motor_A,127);
	pinMode(motor_B,OUTPUT);
	digitalWrite(motor_B,LOW);
	 //turn the PID on
	 myPID.SetMode(AUTOMATIC);
}

void loop() {
	// put your main code here, to run repeatedly:
	Input = MAX_encode_num;
	Setpoint = analogRead(VRX)/20;
	double gap = abs(Setpoint-Input);
	//Serial.println(tc1.MAX_time);
	
	myPID.SetTunings(consKp, consKi, consKd);
	
	myPID.Compute();
	
	analogWrite(motor_A, Output);
}

void report_encode()
{
	//编码器结账
	MAX_encode_num = encode_num;
	encode_num = 0;
	Serial.print("A:");
	Serial.print(Setpoint);
	Serial.print("B:");
	Serial.print(MAX_encode_num);
	Serial.print("kp:");
	Serial.print(consKp);
	Serial.print("ki:");
	Serial.print(consKi);
	Serial.print("kd:");
	Serial.println(consKd);
}


void increase()
{
	encode_num++;
}
void serialEvent(){
	String str = Serial.readStringUntil('\n');
	str.toLowerCase();
	
	//int pos = str.indexOf('\n');
	//String str1 = str.substring(0,pos);
	
	int pos_kp = str.indexOf("kp:");
	int pos_ki = str.indexOf("ki:");
	int pos_kd = str.indexOf("kd:");
	
	String str_kp = str.substring(pos_kp+3,pos_ki);
	String str_ki = str.substring(pos_ki+3,pos_kd);
	String str_kd = str.substring(pos_kd+3);
	
	consKp = str_kp.toDouble();
	consKi = str_ki.toDouble();
	consKd = str_kd.toDouble();
	
}