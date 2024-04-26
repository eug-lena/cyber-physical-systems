#include "PrimeChecker.hpp"

// This method checks if the given number is a prime number or not
bool PrimeChecker::isPrime(uint16_t n) {
    bool retVal{true};
    if (n<2 || 0 == n%2) {
        retVal = false;
    }
    else {
        for(uint16_t i{3}; (i*i) <= n; i += 2) {
            if (0 == n%i) {
                return false; // this returns false
                break;
            }
        }
    }
    return retVal;
}
