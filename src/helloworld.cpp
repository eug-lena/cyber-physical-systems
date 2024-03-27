#include <iostream>			// Include a input/output library
#include "PrimeChecker.hpp"	// Include a header file for the program checking if a number is prime

int main(int argc, char** argv) {
	// Check if the number of arguments is correct
	if (argc == 2) {
		// Convert the string received into an integer value
		int number = std::stoi(argv[1]);
		// Declare a primechecker variable
		PrimeChecker pc;
		// Print out a message to the console
		std::cout << "Group, 18; " << number << " is a prime number? " << pc.isPrime(number) << std::endl;
	}
	std::cout << "Thanks for using the program. Good bye :)\n" << std::endl;
	// Exit the program
	return 0;
}
