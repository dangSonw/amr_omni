#ifndef KALMAN_H
#define KALMAN_H

class ScalarKalman {
public:
    ScalarKalman(float process_noise, float measurement_noise);

    void reset(float estimate, float covariance);
    float update(float measurement, float delta_sec);
    float estimate() const;

private:
    float process_noise_;
    float measurement_noise_;
    float estimate_;
    float covariance_;
    bool initialized_;
};

#endif