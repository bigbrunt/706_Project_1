#ifndef VECTOR_H
#define VECTOR_H

#include <Arduino.h> // Required for float, size_t

class Vector {
private:
    float* data;         // Pointer to dynamically allocated array
    size_t capacity;     // Current capacity of the array
    size_t currentSize;  // Number of elements stored

    // Helper function to resize the array
    void resize(size_t newCapacity);

public:
    // Constructor & Destructor
    Vector();
    ~Vector();

    // Add a float value to the vector
    void add(float value);

    // Return the current size of the vector
    size_t size() const;

    // Return the index of the smallest value
    int indexOfSmallest() const;
};

#endif // VECTOR_H
