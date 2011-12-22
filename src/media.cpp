#include "media_core.hpp"
#include "media_transport.hpp"
#include "media_audio.hpp"

#ifdef __MACH__
#include <mach/mach_init.h>
#include <mach/thread_policy.h>
#include <mach/thread_act.h>
#include <mach/mach_time.h>
#endif

extern "C" {
#include "../ilbc/iLBC_encode.h"
#include "../ilbc/iLBC_decode.h"
}

#include <memory>
#include <boost/thread.hpp> // for some reason there are no boost::thread on Mac OS X

namespace Media {

boost::asio::io_service g_io;

namespace Transport {
boost::asio::io_service g_file_io;
std::unique_ptr<boost::asio::io_service::work> g_file_work;
boost::thread g_file_thread;
}

namespace Audio {

template<size_t N>
IlbcDecState<N>& transform(IlbcFrame<N> const& in, LinearFrame<N>& out, IlbcDecState<N>&& state) {
  float o[N];
  iLBC_decode(o, in.d_->begin(), reinterpret_cast<iLBC_Dec_Inst_t*>(&state), in.d_ ? 1 : 0);

  mutable_buffers(out);
  std::copy(o, o + N, out.d_->begin()); 

  return state;
}

template IlbcDecState<160>& transform(IlbcFrame<160> const&, LinearFrame<160>&, IlbcDecState<160>&&);
template IlbcDecState<240>& transform(IlbcFrame<240> const&, LinearFrame<240>&, IlbcDecState<240>&&);

template<size_t N>
IlbcEncState<N>& transform(LinearFrame<N> const& in, IlbcFrame<N>& out, IlbcEncState<N>&& state) {
  float i[N];

  if(in.d_)
    std::copy(in.d_->begin(), in.d_->end(), i);
  else
    std::fill(i, i + N, 0);  

  iLBC_encode(out.d_->begin(), i, reinterpret_cast<iLBC_Enc_Inst_t*>(&state));

  return state;
}

template IlbcEncState<160>& transform(LinearFrame<160> const&, IlbcFrame<160>&, IlbcEncState<160>&&);
template IlbcEncState<240>& transform(LinearFrame<240> const&, IlbcFrame<240>&, IlbcEncState<240>&&);

}

std::unique_ptr<boost::asio::io_service::work> g_work;
boost::thread g_thread;


void start() {
  Transport::g_file_work = std::unique_ptr<boost::asio::io_service::work>(new boost::asio::io_service::work(Transport::g_file_io));
  Transport::g_file_thread = []() { Transport::g_file_io.run(); };


  g_work = std::unique_ptr<boost::asio::io_service::work>(new boost::asio::io_service::work(g_io));
  g_thread = [](){ g_io.run(); };  
}

void stop() {
  Transport::g_file_work = 0;
  g_work = 0;

  g_thread.join();
  Transport::g_file_thread.join();
}

#if 0
void test1() {
  using namespace Transport;
  using namespace Audio;

  auto d = SharedFunctor<DelayedSource<std::pair<boost::asio::ip::udp::endpoint,UdpPacket>>>::make();
  
  auto r = root(transform_to<RtpPacket>(push(std::move(d), Socket())));
  
  auto s1 = root(transform_to<Packet<AlawFrame<160>>>(branch(std::ref(r)), 8));
  auto s2 = root(transform_to<Packet<AlawFrame<240>>>(branch(std::ref(r)), 8));

  auto s3 = root(rate(jitter(branch(std::ref(s1)))));
  auto s4 = root(encode<AlawFrame<160>>(resize<160>(resize<80>(decode(jitter(branch(std::ref(s2))))))));

  DelayedSource<AlawFrame<160>> d2;

  condition(branch(std::ref(s1)), [&]{ d2 = branch(std::ref(s3));});
  condition(branch(std::ref(s2)), [&]{ d2 = branch(std::ref(s4));});

  auto d3 = SharedFunctor<DelayedSource<std::pair<boost::asio::ip::udp::endpoint,UdpPacket>>>::make();

  auto r2 = push(std::move(d3), Socket());

  *(d3.p_) = transform_to<std::pair<boost::asio::ip::udp::endpoint, RtpPacket>>(transform_to<RtpPacket>(transform_to<Packet<AlawFrame<160>>>(std::move(d2)),5),
    boost::asio::ip::udp::endpoint());
  
  Writer w;
  auto a = push(std::ref(d2), std::move(w));
}
#endif

}

