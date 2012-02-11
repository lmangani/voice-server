#include "../media.hpp"
#include <boost/thread/barrier.hpp>

int main(int argc, char* argv[]) {
  auto address = boost::asio::ip::address::from_string(argv[1]);
  int port = atoi(argv[2]);
  if(!port) throw std::logic_error("port");
  
  Media::start();

  std::shared_ptr<void> guard;
  boost::barrier stop_barrier(2);

  Media::g_io.post([&] {
    using namespace Media;
    using namespace Media::Transport;
    using namespace Media::Audio;
  
    Socket socket;
    socket.s_.open(boost::asio::ip::udp::v4());
    socket.s_.connect(boost::asio::ip::udp::endpoint(address, port));

    auto oneof = [&](boost::system::error_code const&) {
      guard = 0;
      stop_barrier.wait();
    };

    auto r = push(
        transform_to<RtpPacket>(
          transform_to<Packet<AlawFrame<160>>>(
            encode<AlawFrame<160>>(
              rate(read<LinearFrame<160>>(std::move(oneof),STDIN_FILENO)))), 
        8),
      std::move(socket)
    );

    guard = std::make_shared<decltype(r)>(std::move(r));
  });

  stop_barrier.wait();

  std::cerr << "stopping media" << std::endl;
  Media::stop();
}

