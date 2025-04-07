#include "SensorFilter.h"

// Define Kalman filter instance (Q, R, initial estimate, initial error covariance)
SensorFilter kf(0.5, 5, 0, 1, 16705, -1.22, A9);
SensorFilter test(0.5, 5, 0, 1, 1, 1, A9);

void setup() {
    Serial.begin(115200);
}

void loop() {

    int rawValue = analogRead(A9);
    int filteredADC = test.read();
    float distance = 51926 * pow(rawValue, -1.379);
    // Apply Kalman filter
    float filteredDistance = kf.read();

    // Print results
    Serial.print(rawValue);
        // Print results
    Serial.print("\t");
    Serial.print (distance);
    Serial.print("\tKalman Estimate: ");
    Serial.print(filteredDistance);
    Serial.println(" cm");

    delay(100); // Adjust based on required update frequency
}
