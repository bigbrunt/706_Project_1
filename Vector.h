#ifndef VECTOR_H
#define VECTOR_H

class Vector {
private:
    static const int MAX_SIZE = 1000; // Adjust as needed // THIS IS STILL TOO BIG
    float data[MAX_SIZE];
    int currentSize;
    
public:
    Vector(); // Constructor
    void add(float value); // Add a new reading
    int size() const; // Return current size
    int indexOfSmallest() const; // Return index of smallest value
};

#endif
