#include <Arduino.h>


#define MOTOR_MID_VAL 0
#define MOTOR_MIN_VAL -255
#define MOTOR_MAX_VAL 255

class wheel {
  private:
    int IN1;
    int IN2;
    int PWM;
    bool IN1_state;
    bool IN2_state;
    bool left_true = 0;
  public:
    void setLeftTrue(bool T) {
      left_true = T;
    }
    
    int getPin1() {
      return IN1;
    }
    int getPin2() {
      return IN2;
    }

    int getPWM() {
      return PWM;
    }

    void setPin1(int p) {
      IN1 = p;    
    }
    void setPin2(int p) {
      IN2 = p;
    }

    void setPWM(int PWM_pin) {
      PWM = PWM_pin;
    }

    // Setup pins
    void setup(int p1, int p2, int PWM_pin);
    // Set forward, set back / switch direction 
    // Write analog PWM to enable pin
    void changeDirection(bool state);
    void writeToMotor(int val);      
};
