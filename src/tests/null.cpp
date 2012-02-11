#include "../media.hpp"

Media::Audio::LinearFrame<80> null_source() {
  return Media::Audio::LinearFrame<80>();
}

int main(int argc, char* argv[]) {
  
  Media::start();

  std::shared_ptr<void> guard;

  Media::g_io.post([&] {
    using namespace Media;
    using namespace Media::Transport;
    using namespace Media::Audio;
    
    auto r = push(rate(pull(std::bind(null_source))), Writer(STDOUT_FILENO));
    guard = std::make_shared<decltype(r)>(std::move(r)); 
  });

  pause();

  Media::stop();
}

