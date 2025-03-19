#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

//ensure you #include "KalmanFilter.h"
//to include in main - define e.g. KalmanFilter filter(0.01, 5, 0, 1);
// // Apply Kalman filter - float filteredDistance = fliter.update(distance);


class SensorFilter {
  private:
    float x_est; // Estimated state
    float P_est; // Estimated error covariance
    float Q;     // Process noise covariance
    float R;     // Measurement noise covariance

  public:
    SensorFilter(float processNoise, float measurementNoise, float initialEstimate = 0, float initialError = 1) {
        Q = processNoise;
        R = measurementNoise;
        x_est = initialEstimate;
        P_est = initialError;
    }

    float update(float raw_distance) {
        // Prediction step
        float x_pred = x_est;
        float P_pred = P_est + Q;

        // Update step
        float K = P_pred / (P_pred + R);
        x_est = x_pred + K * (raw_distance - x_pred);
        P_est = (1 - K) * P_pred;

        return x_est;
    }
};

#endif
