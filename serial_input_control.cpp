#include "serial_input_control.h"

void serial_input_control(Stream &S) {
// pins 2,3,4,5,6,7 are connected to ENB, IN4, IN3, IN2, IN1, ENA respectively on the rear motor driver
// 
// pins 8,9,10,11,12,13 are connected to ENB, IN4, IN3, IN2, IN1, ENA respectively on forward motor driver
// IN4 and IN3 control the direction of the left side motors (respective to the raspi camera)
// IN1 and IN2 control the right side motors. 
// All above pins are PWM on the Arduino Mega
// Run the cam_test.py from the desktop using python3 to see through the raspi camera
// 

// It is possible to set up communication between the pi and the arduino.Receiving strings from arduino is available under "serial_test.py" 
// RUn it wihle the arduino is using "serial.print". ENSURE THE SERIAL MONITOR IS NOT OPEN
// https://www.jaycar.com.au/arduino-compatible-stepper-motor-controller-module/p/XC4492 These are the motors drivers being used. The specifications
// tab has datasheets on how to use. 
// An MPU6050 is connected on the I2C bus. It can output accel + gyro data. 
/*
  digitalWrite(IN1_1, LOW);
  digitalWrite(IN2_1, HIGH);
  digitalWrite(IN3_1, LOW);
  digitalWrite(IN4_1, HIGH);

  digitalWrite(IN1_2, HIGH);
  digitalWrite(IN2_2, LOW);
  digitalWrite(IN3_2, LOW);
  digitalWrite(IN4_2, HIGH); 
 */



	int ENB_1 = 2;
	int IN1_1 = 6;
	int IN2_1 = 5;
	int IN3_1 = 4;
	int IN4_1 = 3;
	int ENA_1 = 7;


	int ENA_2 = 13;
	int IN1_2 = 12;
	int IN2_2 = 11;
	int IN3_2 = 10;
	int IN4_2 = 9;
	int ENB_2 = 8;

	int val = 0;
	
	if (S.available()) {
		char bite = S.read();
    if (bite == 'g') {
		S.println("GO");
		val+= 10;
		analogWrite(ENA_1, val);
		analogWrite(ENB_1, val);
		analogWrite(ENA_2, val);
		analogWrite(ENB_2, val);
    }
    if (bite == 's') {
		S.println("STOP");
		val-= 10;
		analogWrite(ENA_1, val);
		analogWrite(ENB_1, val);
		analogWrite(ENA_2, val);
		analogWrite(ENB_2, val);
  
    }
    if (bite == 'q') {
		analogWrite(ENA_1, 0);
		analogWrite(ENB_1, 0);
		analogWrite(ENA_2, 0);
		analogWrite(ENB_2, 0);
    }
    if (bite == 'a') {
		digitalWrite(IN1_1, HIGH);
		digitalWrite(IN2_1, LOW);
		digitalWrite(IN3_1, HIGH);
		digitalWrite(IN4_1, LOW);

		digitalWrite(IN1_2, LOW);
		digitalWrite(IN2_2, HIGH);
		digitalWrite(IN3_2, HIGH);
		digitalWrite(IN4_2, LOW);
    }
    if (bite == 'd') {
		digitalWrite(IN1_1, LOW);
		digitalWrite(IN2_1, HIGH);
		digitalWrite(IN3_1, LOW);
		digitalWrite(IN4_1, HIGH);

		digitalWrite(IN1_2, HIGH);
		digitalWrite(IN2_2, LOW);
		digitalWrite(IN3_2, LOW);
		digitalWrite(IN4_2, HIGH);
    }
    if (bite == 'f') {
		analogWrite(ENA_1, 255);
		analogWrite(ENB_1, 255);
		analogWrite(ENA_2, 255);
		analogWrite(ENB_2, 255);
    }
    if (bite == 'w') {
		digitalWrite(IN1_1, HIGH);
		digitalWrite(IN2_1, LOW);
		digitalWrite(IN3_1, LOW);
		digitalWrite(IN4_1, HIGH);

		digitalWrite(IN1_2, LOW);
		digitalWrite(IN2_2, HIGH);
		digitalWrite(IN3_2, LOW);
		digitalWrite(IN4_2, HIGH);
    }
  }
}
