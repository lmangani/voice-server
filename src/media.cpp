#include "media.hpp"

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

#include <boost/thread.hpp>

namespace Media {

struct empty_fun {
  template<typename... Args>
  void operator()(Args... args) {}
};

boost::asio::io_service g_io;

namespace Transport {

void Socket::operator()(UdpPacket const& p) {
  socket_->async_send(boost::asio::const_buffers_1(p.data_.get(), p.length_), boost::bind<void>(empty_fun(), p, _1, _2));
}

struct RtpHeader {
  unsigned cc:4;      // CSRC count
  unsigned x:1;       // extension flag
  unsigned p:1;       // padding flag
  unsigned version:2; // RTP version
  unsigned pt:7;      // payload type
  unsigned m:1;       // marker
  unsigned seq:16;    // sequence number
  boost::uint32_t timestamp;
  boost::uint32_t ssrc;      // synchronization source
};

bool parse(UdpPacket const& in, RtpPacket & out) {
  if(in.length_ < sizeof(RtpHeader))
    return false;

  RtpHeader* hdr = reinterpret_cast<RtpHeader*>(in.data_.get());
  
  size_t offset = sizeof(*hdr) + sizeof(boost::uint32_t) * hdr->cc;
  if(offset > in.length_) return false;
  
  if(hdr->x)
    offset += ntohs(*reinterpret_cast<boost::uint16_t const*>(hdr+1));

  if(offset > in.length_) return false;

  size_t padding = hdr->p ? *(reinterpret_cast<boost::uint8_t const*>(hdr) + in.length_ - 1) : 0;

  if(offset + padding > in.length_) return false;

  out.pt_ = hdr->pt;
  out.marker_ = hdr->m;
  out.seq_ = ntohs(hdr->seq);
  out.ts_ = ntohl(hdr->timestamp);
  out.ssrc_ = ntohl(hdr->ssrc);
  out.payload_ = boost::shared_ptr<void>(in.data_, reinterpret_cast<boost::uint32_t*>(hdr+1)+hdr->cc);
  out.length_ = in.length_ - offset - padding;
  
  return true;
}

struct ArrayDeleter {
  template<typename T>
  void operator()(T* t) {
    delete[] t;
  }
};

bool parse(RtpPacket const& in, UdpPacket& out) {
  boost::shared_ptr<void> p(new char[sizeof(RtpHeader) + in.length_], ArrayDeleter());

  RtpHeader* hdr = reinterpret_cast<RtpHeader*>(p.get());
  memset(hdr, 0, sizeof(*hdr));

  hdr->pt = in.pt_;
  hdr->m = in.marker_;
  hdr->seq = htons(in.seq_);
  hdr->timestamp = htonl(in.ts_);
  hdr->ssrc = htonl(in.ssrc_);
  
  memcpy(hdr+1, in.payload_.get(), in.length_);

  out.data_ = p;
  out.length_ = sizeof(*hdr) + in.length_;   

  return true;
}

}

namespace Audio {

namespace Detail {
IlbcDecoder::IlbcDecoder(int mode) : state_(boost::make_shared<iLBC_Dec_Inst_t>()) {
  initDecode(static_cast<iLBC_Dec_Inst_t*>(state_.get()), mode, 1); 
}

void IlbcDecoder::operator()(boost::uint8_t const* in, boost::int16_t* out) {
  float data[240];
  iLBC_decode(data, const_cast<boost::uint8_t*>(in), static_cast<iLBC_Dec_Inst_t*>(state_.get()), (in ? 1 : 0));

  std::copy(data, data + static_cast<iLBC_Dec_Inst_t*>(state_.get())->mode*8, out);    
}

IlbcEncoder::IlbcEncoder(int mode) : state_(boost::make_shared<iLBC_Enc_Inst_t>()) {
  initEncode(static_cast<iLBC_Enc_Inst_t*>(state_.get()), mode); 
}

void IlbcEncoder::operator()(boost::int16_t const* in, boost::uint8_t* out) {
  float data[240];
  std::copy(in, in + static_cast<iLBC_Dec_Inst_t*>(state_.get())->mode*8, data);

  iLBC_encode(out, data, static_cast<iLBC_Enc_Inst_t*>(state_.get()));
}

}

}

std::auto_ptr<boost::asio::io_service::work> g_work;
boost::thread g_thread;


void start() {
  g_work = std::auto_ptr<boost::asio::io_service::work>(new boost::asio::io_service::work(g_io));
  g_thread = boost::thread(boost::bind(static_cast<std::size_t (boost::asio::io_service::*)()>(&boost::asio::io_service::run), boost::ref(g_io)));  
}

void stop() {
  //g_work.reset();

  //g_thread.join();
}

}

