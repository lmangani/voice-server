#ifndef __MEDIA_TRANSPORT_HPP__
#define __MEDIA_TRANSPORT_HPP__

#include "media_core.hpp"

namespace Media {namespace Transport{

extern boost::asio::io_service g_file_io;

struct RtpHeader {
  unsigned cc:4;      // CSRC count
  unsigned x:1;       // extension flag
  unsigned p:1;       // padding flag
  unsigned version:2; // RTP version
  unsigned pt:7;      // payload type
  unsigned m:1;       // marker
  unsigned seq:16;    // sequence number
  boost::uint32_t ts; // timestamp
  boost::uint32_t ssrc;      // synchronization source
};

struct UdpPacket {
  UdpPacket() {}
  UdpPacket(std::shared_ptr<void> const& d, size_t l) : data_(d), length_(l) {}

  std::shared_ptr<void> data_;
  size_t length_;

  std::shared_ptr<RtpHeader> hdr_;
};

struct RtpPacket : UdpPacket {};

inline
void transform(UdpPacket const& f, std::pair<bool, RtpPacket>& t) {
  t.first = false;
  if(f.length_ <= sizeof(RtpHeader)) return;
  
  RtpHeader* hdr = reinterpret_cast<RtpHeader*>(f.data_.get());
  if(hdr->version != 2) return;
  if(f.length_ <= sizeof(RtpHeader) + hdr->cc*sizeof(std::uint32_t)) return;

  t.second.hdr_ = std::shared_ptr<RtpHeader>(f.data_, hdr);
  t.second.data_ = std::shared_ptr<void>(f.data_, reinterpret_cast<std::uint32_t*>(hdr + 1) + hdr->cc);
  t.first = true;
}

inline
std::array<boost::asio::const_buffer, 2> const_buffers(UdpPacket const& p) {
  return std::array<boost::asio::const_buffer, 2>{{boost::asio::const_buffer(p.data_.get(), p.length_), 
    boost::asio::const_buffer(p.hdr_.get(), p.hdr_ ? sizeof(RtpHeader) + sizeof(std::uint32_t)*p.hdr_->cc : 0)}};
}

struct Socket {
  typedef UdpPacket source_type;

  Socket() : s_(g_io) {}

  Socket(Socket&& s) : s_(static_cast<boost::asio::ip::udp::socket&&>(s.s_)) {
    s_.cancel();
  }

  template<typename C> 
  void operator()(C c) {
    auto p = std::make_shared<std::array<std::uint8_t, 1500>>();
    s_.async_receive(boost::asio::mutable_buffers_1(p->begin(), p->size()),
      [=](boost::system::error_code const& ec, size_t bytes) mutable {
        if(!ec) c(UdpPacket(p, bytes));
      }
    );
  }

  void operator()(std::pair<boost::asio::ip::udp::endpoint, UdpPacket> const& d) {
    s_.async_send_to(const_buffers(d.second), d.first, std::bind(Nop(), d.second, _1, _2));
  }

  boost::asio::ip::udp::socket s_;
};

template<typename T>
inline
boost::asio::ip::udp::endpoint const& transform(T const& t, std::pair<boost::asio::ip::udp::endpoint, T>& o,
  boost::asio::ip::udp::endpoint const& a = boost::asio::ip::udp::endpoint()) {
  o = std::make_pair(a, t);
  return a;
}


template<typename F>
struct Reader {
  typedef F source_type;

  template<typename... Args>
  Reader(Args&&... args) : fd_(g_file_io, std::forward<Args>(args)...) {}
  
  Reader(Reader&& r) : fd_(std::forward<boost::asio::posix::stream_descriptor>(r.fd_)) {}

  template<typename C>
  void operator()(C c) {
    F f;
    async_read(fd_, mutable_buffers(f), [=](boost::system::error_code const& ec, size_t bytes) mutable {
      g_io.post([=]() mutable {c(f);});
    });
  }

  boost::asio::posix::stream_descriptor fd_;
};


struct Writer {
  template<typename... Args>
  Writer(Args... args) : fd_(g_file_io, args...) {}
  Writer(Writer&& w) : fd_(std::forward<boost::asio::posix::stream_descriptor>(w.fd_)) {}

  template<typename F>
  void operator()(F const& f) {
    boost::asio::async_write(fd_, const_buffers(f), [f](boost::system::error_code const&, size_t){});
  }

  boost::asio::posix::stream_descriptor fd_;
};


}}

#endif
