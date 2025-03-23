#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

//ensure you #include "SensorFilter.h"
//to include in main - define e.g. SensorFilter filter(0.01, 5, 0, 1);
// // Apply Kalman filter - float filteredDistance = fliter.update(distance);


class SensorFilter {
  private:
    float x_est; // Estimated state
    float P_est; // Estimated error covariance
    float Q;     // Process noise covariance
    float R;     // Measurement noise covariance
    float coefficent;
    float exponent;
    int pin_num;

  public:
    SensorFilter(float processNoise, float measurementNoise, float initialEstimate, float initialError, float co, float exp, int pin) {
        Q = processNoise;
        R = measurementNoise;
        x_est = initialEstimate;
        P_est = initialError;
        coefficent = co;
        exponent = exp;
        pin_num = pin;
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

    float read(){
      int rawValue = analogRead(pin_num);
    
      float distance = coefficent * pow(rawValue, exponent);

      if (distance < 4) distance = 4;
      if (distance > 1000) distance = 1000;

      // Apply Kalman filter
      float filteredDistance = update(distance);
      return filteredDistance;
    }



};

#endif
