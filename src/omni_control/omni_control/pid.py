import math


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class ScalarKalman:
    """Small allocation-free scalar Kalman filter for embedded-like loops."""

    def __init__(self, process_noise, measurement_noise):
        if process_noise < 0 or measurement_noise <= 0:
            raise ValueError('Kalman noise parameters are invalid')
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        self.estimate_value = 0.0
        self.covariance = 1.0
        self.initialized = False

    def reset(self, estimate=0.0, covariance=1.0):
        if not math.isfinite(estimate) or covariance <= 0:
            raise ValueError('Kalman reset values are invalid')
        self.estimate_value = float(estimate)
        self.covariance = float(covariance)
        self.initialized = True

    def update(self, measurement, delta_sec):
        if not math.isfinite(measurement):
            raise ValueError('Kalman measurement must be finite')
        delta_sec = max(float(delta_sec), 1e-6)
        if not self.initialized:
            self.reset(measurement)
            return self.estimate_value

        self.covariance += self.process_noise * delta_sec
        gain = self.covariance / (self.covariance + self.measurement_noise)
        self.estimate_value += gain * (measurement - self.estimate_value)
        self.covariance *= 1.0 - gain
        return self.estimate_value


class WheelSpeedPid:
    """PID speed loop whose output is a bounded motor correction."""

    def __init__(self, kp, ki, kd, output_limit, integral_limit=None):
        self.configure(kp, ki, kd, output_limit, integral_limit)
        self.reset()

    def configure(self, kp, ki, kd, output_limit, integral_limit=None):
        values = (kp, ki, kd, output_limit)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('PID parameters must be finite')
        if kp < 0 or ki < 0 or kd < 0 or output_limit <= 0:
            raise ValueError('PID parameters are invalid')
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = float(output_limit)
        self.integral_limit = (float(integral_limit) if integral_limit is not None
                               else self.output_limit)
        if self.integral_limit <= 0:
            raise ValueError('PID integral limit must be positive')

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def update(self, setpoint, measurement, delta_sec):
        values = (setpoint, measurement, delta_sec)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('PID values must be finite')
        delta_sec = max(float(delta_sec), 1e-6)
        error = float(setpoint) - float(measurement)
        if not self.initialized:
            self.previous_error = error
            self.initialized = True
        derivative = (error - self.previous_error) / delta_sec
        candidate_integral = self.integral + error * delta_sec
        candidate_integral = _clamp(candidate_integral,
                                    -self.integral_limit,
                                    self.integral_limit)
        output = (self.kp * error + self.ki * candidate_integral +
                  self.kd * derivative)
        output = _clamp(output, -self.output_limit, self.output_limit)
        # Do not continue integrating while the output is saturated in the
        # same direction as the error.
        if (abs(output) < self.output_limit or
                output * error < 0.0):
            self.integral = candidate_integral
        self.previous_error = error
        return output