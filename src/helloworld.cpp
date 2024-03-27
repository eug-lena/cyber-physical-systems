#include <iostream>			// Include a input/output library
#include "PrimeChecker.hpp"	// Include a header file for the program checking if a number is prime

int main(int argc, char** argv) {
	if (argc == 2) {
		int number = std::stoi(argv[1]);
		PrimeChecker pc;
		std::cout << "Group, 18; " << number << " is a prime number? " << pc.isPrime(number) << std::endl;
	}
	return 0;
}
