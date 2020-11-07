#include "Drive.h"

Drive::Drive() :targetX(0), targetY(0), remainderX(0), remainderY(0) {};

Drive::~Drive(){

}

void Drive::motor_control(int frontleft, int frontright, int rearleft, int rearright) {
// inputs should be from -255 to 255, negative for reverse direction
	
// pins 2,3,4,5,6,7 are connected to ENB, IN4, IN3, IN2, IN1, ENA respectively on the rear motor driver
// pins 8,9,10,11,12,13 are connected to ENB, IN4, IN3, IN2, IN1, ENA respectively on forward motor driver
// IN4 and IN3 control the direction of the left side motors (respective to the raspi camera)
// IN1 and IN2 control the right side motors. 
// All above pins are PWM on the Arduino Mega

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
	
	// Set motor directions depending on the signs of the inputs
	digitalWrite(IN1_1, frontright >= 0 ? HIGH : LOW);
	digitalWrite(IN2_1, frontright < 0 ? HIGH : LOW);
	digitalWrite(IN3_1, frontleft >= 0 ? HIGH : LOW);
	digitalWrite(IN4_1, frontleft < 0 ? HIGH : LOW);
	
	digitalWrite(IN1_2, rearright >= 0 ? HIGH : LOW);
	digitalWrite(IN2_2, rearright < 0 ? HIGH : LOW);
	digitalWrite(IN3_2, rearleft >= 0 ? HIGH : LOW);
	digitalWrite(IN4_2, rearleft < 0 ? HIGH : LOW);
	
	// Set motor speeds depending on the magnitudes of the inputs
	
	analogWrite(ENA_1, abs(frontright));
	analogWrite(ENB_1, abs(frontleft));
	analogWrite(ENA_2, abs(rearright));
	analogWrite(ENB_2, abs(rearleft));

}

bool Drive::waypoint(double x, double y, int dir, DeadReckoner& DeadReckoner) {
	// drives bot to waypoint in a straight line by the coord specified
	// setting target only if previous target has been reached

	if (abs(DeadReckoner.getX() - targetX) <= 0 && abs(DeadReckoner.getY() - targetY) <= 0) {
		targetX = x;
		targetY = y;
	}
	
	// convert to correct reference for angle and take difference to heading
	// putting this line here so rotate_angle is updated everytime function is called, not necessary, rotate_angle only needs to be set once; errors might fluctuate
	double rotate_angle = DeadReckoner.getHeading() - (-atan2(abs(targetY-DeadReckoner.getY()), abs(targetX-DeadReckoner.getX())) * 180.0 / PI + 90); 
	if(rotate(rotate_angle, DeadReckoner)) {
		double distance = sqrt((targetY-DeadReckoner.getY())*(targetY-DeadReckoner.getY()) + (targetX-DeadReckoner.getX())*(targetX-DeadReckoner.getX()));
		bool there = linear(distance, dir);
		if(there == 1)
			return 1;
	}
	return 0;
}

bool Drive::rotate(double rotate_angle, DeadReckoner& DeadReckoner) {
	// rotates bot on the "spot" by amount specified from current pose
	// returns 1 when reached
	// run function in a loop
	
	// setting target only if previous angle has been reached
	if (abs(DeadReckoner.getHeading() - targetHeading) <= 0) {
		targetHeading = DeadReckoner.getHeading() + rotate_angle;
		motor_control(0,0,0,0);
		return 1;
	}
	else {
		double HeadingError = targetHeading - DeadReckoner.getHeading();
		double proportion = ((HeadingError > 0) - (HeadingError < 0));
		motor_control(255*proportion,255*proportion,-255*proportion,-255*proportion);
	}
	
	return 0;
}

bool Drive::linear(int distance, int dir) {
	// moves bot forwards or backwards
	// returns 1 when reached
	// run function in a loop
	
	if(abs(distance)>0) {
		motor_control(255,255,255,255);
		return 0;
	}
	else {
		motor_control(0,0,0,0);
		return 1;
	}
	

}
