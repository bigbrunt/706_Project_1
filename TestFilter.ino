#include "SensorFilter.h"

// Define Kalman filter instance (Q, R, initial estimate, initial error covariance)
SensorFilter kf(0.5, 5, 0, 1, 3123, -0.9, A7);
SensorFilter test(0.5, 5, 0, 1, 1, 1, A7);

void setup() {
    Serial.begin(115200);
}

void loop() {

    int rawValue = analogRead(A7);
    int filteredADC = test.read();
    float distance = 3123 * pow(rawValue, -0.9);
    // Apply Kalman filter
    float filteredDistance = kf.read();

    // Print results
    Serial.print(filteredADC);
        // Print results
    Serial.print("\t");
    Serial.print (distance);
    Serial.print("\tKalman Estimate: ");
    Serial.print(filteredDistance);
    Serial.println(" cm");

    delay(100); // Adjust based on required update frequency
}

