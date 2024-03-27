#include <iostream>
#include "PrimeChecker.hpp"

int main(int argc, char** argv) {
	if (argc == 2) {
		int number = std::stoi(argv[1]);
		PrimeChecker pc;
		std::cout << "Group, 18; " << number << " is a prime number? " << pc.isPrime(number) << std::endl;
	}
	std::cout << "Thanks for using the program. Good bye :)\n" << std::endl;
	return 0;
}
