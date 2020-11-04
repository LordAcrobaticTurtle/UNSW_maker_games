#include "wheel.h"

void wheel::setup(int p1, int p2, int PWM_pin) {
  setPin1(p1);
  setPin2(p2);
  setPWM(PWM_pin);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  IN1_state = true;
  IN2_state = false;
  digitalWrite(IN1, IN1_state);
  digitalWrite(IN2, IN2_state);
  left_true = true;
  analogWrite(PWM, 0);
}

// True/false values dictate direction of spin
void wheel::changeDirection(bool state) {
  if (left_true) {
    IN1_state = state;
    IN2_state = !state;
  } else {
    IN1_state = !state;
    IN2_state = state;
  }
  digitalWrite(IN1, IN1_state);
  digitalWrite(IN2, IN2_state);
}

void wheel::writeToMotor(int val) {
  // Value is ranged between 0 and 500
  if (val >= MOTOR_MID_VAL) {
    changeDirection(true);
    int mapped = map(val,MOTOR_MID_VAL, MOTOR_MAX_VAL, 0, 255);
    mapped = constrain(mapped, 0, 255);
    //Serial.println(String(mapped) +" "+ String(PWM));
    analogWrite(PWM, mapped);
  } else if (val < MOTOR_MID_VAL) {
    changeDirection(false);
    int mapped = map(val,MOTOR_MID_VAL, MOTOR_MIN_VAL, 0, 255);
    mapped = constrain(mapped, 0, 255);
    //Serial.println(String(mapped) +" "+ String(PWM));
    analogWrite(PWM, mapped);
  }
}
