#ifndef PID_H
#define PID_H

class WheelSpeedPid {
public:
    WheelSpeedPid(float kp, float ki, float kd, float output_limit);

    void configure(float kp, float ki, float kd, float output_limit);
    void reset();
    float update(float setpoint, float measurement, float delta_sec);

private:
    float kp_;
    float ki_;
    float kd_;
    float output_limit_;
    float integral_limit_;
    float integral_;
    float previous_error_;
    bool initialized_;
};

#endif