#ifndef RINGBUF_H
#define RINGBUF_H

#include <Arduino.h> // Required for pinMode(), analogRead(), and pow()

#define bufferSize 400 // Adjust size as needed

class RingBuf {
private:
    int buffer[bufferSize];  // Array to hold the buffer
    int front;               // Index for the front of the buffer
    int rear;                // Index for the rear of the buffer
    int size;                // Current number of elements in the buffer
    int pin;                 // Pin number
    double coefficent;       // Coefficient for ADC -> cm equation
    double exponent;         // Exponent for ADC -> cm equation
    long sum;                // Sum of the elements in the buffer (long for large sums)

public:
    // Constructors
    RingBuf();
    RingBuf(int p, double c, double e);

    // Buffer initialization
    void initialise_buf();

    // Add a new value from the analog pin
    void push();

    // Calculate the moving average
    int32_t movingAverage();
};

#endif // RINGBUF_H
