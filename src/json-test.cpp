#include <iostream>
#include "json.hpp"

int main(int argc, char* argv[]) {
  std::cout << std::boolalpha << JSON::parse(std::cin);
}

