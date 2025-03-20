#include "Vector.h"

// Initial capacity for dynamic allocation
#define INITIAL_CAPACITY 10

// Constructor: Initialize an empty vector
Vector::Vector() : capacity(INITIAL_CAPACITY), currentSize(0) {
    data = new float[capacity];
}

// Destructor: Free allocated memory
Vector::~Vector() {
    delete[] data;
}

// Resize the array to a new capacity
void Vector::resize(size_t newCapacity) {
    float* newData = new float[newCapacity];
    for (size_t i = 0; i < currentSize; ++i) {
        newData[i] = data[i];
    }
    delete[] data;
    data = newData;
    capacity = newCapacity;
}

// Add a float value to the vector (resize if needed)
void Vector::add(float value) {
    if (currentSize == capacity) {
        resize(capacity * 2); // Double the capacity when full
    }
    data[currentSize++] = value;
}

// Return the current size of the vector
size_t Vector::size() const {
    return currentSize;
}

// Return the index of the smallest value
int Vector::indexOfSmallest() const {
    if (currentSize == 0) {
        return -1; // Return -1 if the vector is empty
    }

    int minIndex = 0;
    for (size_t i = 1; i < currentSize; ++i) {
        if (data[i] < data[minIndex]) {
            minIndex = i;
        }
    }
    return minIndex;
}
