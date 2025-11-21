#include <iostream>
#include <cmath>

// Calculates the Triple Modular Redundancy (TMR) value from three sensor readings.
// It rounds the inputs, compares their differences, and returns the average of the closest pair.
// If all differences are equal, it returns the average of all three values.
float TMR(float a, float b, float c);

int main() {
    // Replace this with some actual way to get the data
    float dataA = 2;
    float dataB = 2.003;
    float dataC = 2.004;
   
    float dataTMR = TMR(dataA, dataB, dataC);
    std::cout << dataTMR << std::endl;
  
    return 0;
}                                    

float TMR(float a, float b, float c) {
    // Change std namespace to something else lol
    float tempA = static_cast<float>(std::round(a));
    float tempB = static_cast<float>(std::round(b));
    float tempC = static_cast<float>(std::round(c));
    
    // Find relative differences
    float abDiff = std::abs(tempA - tempB);
    float acDiff = std::abs(tempA - tempC);
    float bcDiff = std::abs(tempB - tempC);
    
    // Compare relative differences and find the least different pair
    if((abDiff < bcDiff) && (abDiff < acDiff)){
        // set sensor C to (A+B)/2
        return (a+b)/2;
    }else if((acDiff < bcDiff) && (acDiff < abDiff)){
        // set sensor B to (A+C)/2
        return (a+c)/2;
    }else if((bcDiff < abDiff) && (bcDiff < acDiff)){
        // set sensor A to (B+C)/2
        return (b+c)/2;
    }else{
        return (a+b+c)/3;
    }
}