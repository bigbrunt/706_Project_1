#include "SensorFilter.h"

// Define Kalman filter instance (Q, R, initial estimate, initial error covariance)
SensorFilter kf(0.01, 5, 0, 1);

void setup() {
    Serial.begin(115200);
}

void loop() {
    // Read raw sensor value
    int rawValue = analogRead(A4);
    
    // Convert raw value to distance (assuming Sharp GP2Y0A21YK0F)
    float voltage = rawValue * (5.0 / 1023.0);
    // Prevent division by zero by ensuring voltage is never zero
    if (voltage < 0.01) voltage = 0.01;
    float distance = 27.86 * pow(voltage, -1.15);

    // Apply Kalman filter
    float filteredDistance = kf.update(distance);

    // Print results
    Serial.print("Raw Distance: ");
    Serial.print(distance);
    Serial.print(" cm\tKalman Estimate: ");
    Serial.print(filteredDistance);
    Serial.println(" cm");

    delay(100); // Adjust based on required update frequency
}
