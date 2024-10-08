/*
    SPEED INPUT 0 -> 1023
    SPEED OUTPUT 0 -> 255

    ALl encoders are connected to the first interrupt pin,
    Each encoder is connected on a digital pin,

*/


#include <INCLUDE_1.h>


enum Clamp { NO_CLAMP, CLAMP };
enum Direction { CW, CCW };
volatile unsigned long long motor_count[4] = {}; //Global variable modified by only ISR, accessed by from the global scope

void ISR_CALLBACK() {   //ISR cannot access class attributes directly, nor can it be a non-static member function
    if(digitalRead(INTERRUPT_FEED1) == HIGH) motor_count[0] += 1;
    if(digitalRead(INTERRUPT_FEED2) == HIGH) motor_count[1] += 1;
    if(digitalRead(INTERRUPT_FEED3) == HIGH) motor_count[2] += 1;
    if(digitalRead(INTERRUPT_FEED4) == HIGH) motor_count[3] += 1;
}

struct Pid {
private:
    float kp, ki, kd, max_output;
    float accum{};
    float prev_err{};
    unsigned long prev_time{};
public:
    Pid(const float kp,const  float ki, const float kd, const float max_output):
            kp(kp), ki(ki), kd(kd), max_output(max_output) {}

    float get_output(const float target, const float current) {
        const float err = target - current;
        const auto dt = (millis() - prev_time) / 1000.0;

        const float p = kp * err;
        const float d = kd * (prev_err - err) / dt;
        accum += err * dt;
        accum = clamp(accum, -max_output, max_output);

        const float i = ki * accum;

        prev_err = err;
        return p + d + i;
    }
};

struct Motor {
private:

    unsigned long prev_count = 0;
    unsigned long prev_time = 0;
    uint8_t pin1;
    uint8_t pin2;
    uint8_t pwm_pin;
    uint8_t interrupt_pin;
    uint8_t speed_pin_in;
    uint8_t direction_pin_in;

    uint8_t count_index;

    float RPM = 0.0;

    void updateRPM() {
        const auto count = motor_count[count_index];
        if(count != this->prev_count) {
            const unsigned long t = millis();
            if(prev_time == t) return;
            this->RPM = ((count - this->prev_count) / ENCODER_ACC) / (t - this->prev_time)  * 60 * 1000;
            this->prev_count = count;
            this->prev_time = t;
        }
    }



public:
    uint32_t max_speed;

    Motor(const uint8_t pin1, const uint8_t pin2, const uint8_t pwm_pin, const uint8_t interrupt_pin,
            const uint8_t speed_pin_in, const uint8_t direction_pin_in, const uint32_t max_speed) :
        pin1(pin1), pin2(pin2), pwm_pin(pwm_pin), interrupt_pin(interrupt_pin), speed_pin_in(speed_pin_in),
            direction_pin_in(direction_pin_in), max_speed(max_speed) {

        pinMode(pwm_pin, OUTPUT);
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
        pinMode(interrupt_pin, INPUT);
    }

    void drive(const Direction direction,const uint8_t speed) const {
        const int l = (direction == CW) ? HIGH : LOW;
        digitalWrite(pin1, l);
        digitalWrite(pin2, !l);
        analogWrite(pwm_pin, speed);
    }
    int getRPM() const {
        return RPM;
    }
};

float softStart(const float target, const float current, const float factor) {
    return factor * current + (1 - factor) * target;
}

uint8_t getPWM(const float speed, const Motor& motor) {
    return map(speed, 0, motor.max_speed, 0, 255);
}






void setup() {
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), ISR_CALLBACK, RISING);
}

void loop() {

    static Motor motor1(MOTOR1_DIR1, MOTOR1_DIR2, MOTOR1_PWM,
      INTERRUPT_FEED1, MOTOR1_SPEED_IN, MOTOR1_DIR_IN, MAX_SPEED1);

    static Motor motor2(MOTOR2_DIR1, MOTOR2_DIR2, MOTOR2_PWM,
        INTERRUPT_FEED2, MOTOR2_SPEED_IN, MOTOR2_DIR_IN, MAX_SPEED2);

    static Pid motor1_pid(3, 0.1, 0.01, 255);
    static Pid motor2_pid(3, 0.1, 0.01, 255);

    constexpr float soft_start_factor = 0.1;

    float desired_speed1 = analogRead(MOTOR1_SPEED_IN);
    float desired_speed2 = analogRead(MOTOR2_SPEED_IN);

    desired_speed1 = map(desired_speed1, 0, 1023, 0, MAX_SPEED1);
    desired_speed2 = map(desired_speed2, 0, 1023, 0, MAX_SPEED2);

    float current_speed1 = motor1.getRPM();
    float current_speed2 = motor2.getRPM();

    float pid_output1 = motor1_pid.get_output(desired_speed1, current_speed1);
    float pid_output2 = motor2_pid.get_output(desired_speed2, current_speed2);

    float soft_start_speed1 = softStart(pid_output1, current_speed1, soft_start_factor);
    float soft_start_speed2 = softStart(pid_output2, current_speed2, soft_start_factor);

    uint8_t pwm_value1 = getPWM(soft_start_speed1, motor1);
    uint8_t pwm_value2 = getPWM(soft_start_speed2, motor2);

    const Direction direction1 = digitalRead(MOTOR1_DIR_IN) == HIGH ? CW : CCW;
    const Direction direction2 = digitalRead(MOTOR2_DIR_IN) == HIGH ? CW : CCW;

    motor1.drive(direction1, pwm_value1);
    motor2.drive(direction2, pwm_value2);

}
