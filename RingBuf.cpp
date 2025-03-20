#include "RingBuf.h"

// Default constructor
RingBuf::RingBuf() : front(0), rear(0), size(0), pin(0), coefficent(1.0), exponent(1.0), sum(0) {}

// Parameterized constructor
RingBuf::RingBuf(int p, double c, double e) : front(0), rear(0), size(0), sum(0) {
    pin = p;
    coefficent = c;
    exponent = e;
    pinMode(pin, OUTPUT);
}

// Initialize the buffer with zeros
void RingBuf::initialise_buf() {
    for (int i = 0; i < bufferSize; i++) {
        buffer[i] = 0;
    }
    front = rear = size = sum = 0;
}

// Push a new value from the analog pin into the buffer
void RingBuf::push() {
    if (size == bufferSize) {
        // If the buffer is full, remove the oldest value from the sum
        sum -= buffer[front];
        front = (front + 1) % bufferSize;  // Move the front pointer forward
    } else {
        size++;
    }

    // Read value, apply the transformation, and add to the buffer
    int value = (int)(coefficent * pow(analogRead(pin), exponent));
    buffer[rear] = value;
    sum += value;

    // Move the rear pointer forward
    rear = (rear + 1) % bufferSize;
}

// Calculate the moving average of the buffer contents
int32_t RingBuf::movingAverage() {
    if (size == 0) {
        return 0;  // Avoid division by zero if the buffer is empty
    }
    return (int32_t)(sum / size);
}
