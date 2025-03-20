#include "Vector.h"

Vector::Vector() : currentSize(0) {}

// Add a new value if there is room
void Vector::add(float value) {
    if (currentSize < MAX_SIZE) {
        data[currentSize++] = value;
    }
}

// Return the current size
int Vector::size() const {
    return currentSize;
}

// Find the index of the smallest value
int Vector::indexOfSmallest() const {
    if (currentSize == 0) return -1; // Handle empty array
    
    int minIndex = 0;
    for (int i = 1; i < currentSize; ++i) {
        if (data[i] < data[minIndex]) {
            minIndex = i;
        }
    }
    return minIndex;
}
