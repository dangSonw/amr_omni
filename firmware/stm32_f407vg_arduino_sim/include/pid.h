#ifndef PID_H
#define PID_H

#include <math.h>
#include <stdint.h>

class SimplePid {
public:
    SimplePid(float kp = 1.0F, float ki = 0.0F, float kd = 0.0F, float output_limit = 1.0F)
        : kp_(kp), ki_(ki), kd_(kd), output_limit_(output_limit),
          integral_(0.0F), previous_measurement_(0.0F), initialized_(false) {}

    void set_tunings(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void get_tunings(float &kp, float &ki, float &kd) const {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }

    void reset() {
        integral_ = 0.0F;
        previous_measurement_ = 0.0F;
        initialized_ = false;
    }

    bool is_passthrough() const {
        return (fabsf(kp_ - 1.0F) < 1e-4F && fabsf(ki_) < 1e-4F && fabsf(kd_) < 1e-4F);
    }

    float update(float setpoint, float measurement, float dt) {
        if (!isfinite(setpoint) || !isfinite(measurement) || dt <= 0.0F) {
            reset();
            return 0.0F;
        }

        // Direct passthrough mode when Kp=1, Ki=0, Kd=0: feedback is zero
        if (is_passthrough()) {
            previous_measurement_ = measurement;
            initialized_ = true;
            return 0.0F;
        }

        const float error = setpoint - measurement;
        if (!initialized_) {
            previous_measurement_ = measurement;
            initialized_ = true;
        }

        // Derivative on measurement to prevent derivative kick: -(y_k - y_{k-1}) / dt
        const float derivative = -(measurement - previous_measurement_) / dt;
        previous_measurement_ = measurement;

        // Anti-windup with conditional integration
        const float candidate_integral = fmaxf(-output_limit_, fminf(output_limit_, integral_ + error * dt));
        float output = kp_ * error + ki_ * candidate_integral + kd_ * derivative;
        output = fmaxf(-output_limit_, fminf(output_limit_, output));

        // Update integral only if not saturating or driving out of saturation
        if (fabsf(output) < output_limit_ || (output * error) < 0.0F) {
            integral_ = candidate_integral;
        }

        return output;
    }

private:
    float kp_;
    float ki_;
    float kd_;
    float output_limit_;
    float integral_;
    float previous_measurement_;
    bool initialized_;
};

#endif  // PID_H

