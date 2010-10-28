#include <iostream>
#include "media.hpp"
#include "json.hpp"

int main(int argc, char* argv[]) {
  try {
    Media::start();
    while(std::cin) {
      std::string s;
      std::getline(std::cin, s);    
    }
    //Media::stop();
  }
  catch(std::exception const& e) {
    std::cerr << "Exception occured: " << e.what() << std::endl;
  }
  return 0;
}
