#include <iostream>
#include "PrimeChecker.hpp"

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
	// Exit the program
	return 0;
}
