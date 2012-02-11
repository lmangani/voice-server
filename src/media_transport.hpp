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
  t.second.length_ = f.length_ - (sizeof(RtpHeader) + hdr->cc*sizeof(std::uint32_t));  
  t.first = true;
}

inline
void transform(RtpPacket const& f, UdpPacket& t) {
  t = f;
}

inline
std::array<boost::asio::const_buffer, 2> const_buffers(UdpPacket const& p) {
  return std::array<boost::asio::const_buffer, 2>{{
    boost::asio::const_buffer(p.hdr_.get(), p.hdr_ ? sizeof(RtpHeader) + sizeof(std::uint32_t)*p.hdr_->cc : 0),
    boost::asio::const_buffer(p.data_.get(), p.length_)}};
}

struct Socket {
  typedef UdpPacket source_type;

  Socket() : s_(g_io) {}

  Socket(Socket&& s) : s_(static_cast<boost::asio::ip::udp::socket&&>(s.s_)) {
    s_.cancel();
  }

  template<typename C> 
  auto operator()(C c) -> decltype(c(UdpPacket(0, 0))) {
    auto p = std::make_shared<std::array<std::uint8_t, 1500>>();
    s_.async_receive(boost::asio::mutable_buffers_1(p->begin(), p->size()),
      [=, this](boost::system::error_code const& ec, size_t bytes) mutable {
        if(!ec) c(UdpPacket(p, bytes));
      }
    );
  }

  void operator()(UdpPacket const& d) {
    s_.async_send(const_buffers(d), [d](boost::system::error_code const& ec, size_t bytes){
    });
  }

  boost::asio::ip::udp::socket s_;
};


template<typename F, typename Cb>
struct Reader {
  typedef F source_type;

  template<typename... Args>
  Reader(Cb&& cb, Args&&... args) : fd_(g_file_io, std::forward<Args>(args)...), cb_(std::make_shared<Cb>(std::forward<Cb>(cb))) {}
  
  Reader(Reader&& r) : fd_(std::move(r.fd_)), cb_(std::move(r.cb_)) {
    fd_.cancel();
  }
  
  Reader& operator=(Reader&& r) {
    fd_.cancel();
    fd_ = std::move(r.fd_);
    cb_ = std::move(r.cb_);
    fd_.cancel();
    return *this;
  }

  template<typename C>
  void operator()(C c) {
    F f;
    std::shared_ptr<C> cptr = std::make_shared<C>(std::move(c));
    std::weak_ptr<C> wptr((cptr));
    p_ = cptr;

    std::weak_ptr<Cb> cb{cb_};

    mutable_buffers(f);
    
    async_read(fd_, mutable_buffers(f), [=](boost::system::error_code const& ec, size_t bytes) mutable {
      if(!ec) {
        g_io.post([=]() mutable {
          std::shared_ptr<C> c2 = wptr.lock();
          if(c2) (*c2)(f);
        });
      }
      else
        g_io.post([=]() {
          std::shared_ptr<Cb> l = cb.lock();
          if(l) (*l)(ec);
        });
    });
  }

  boost::asio::posix::stream_descriptor fd_;
  std::shared_ptr<void> p_;
  std::shared_ptr<Cb> cb_;
};

template<typename F, typename Cb, typename... Args>
Reader<F, Cb> read(Cb&& cb, Args&&... args) {
  return Reader<F,Cb>(std::forward<Cb>(cb), std::forward<Args>(args)...);
}


struct Writer {
  template<typename... Args>
  Writer(Args... args) : fd_(g_file_io, args...) {}
  Writer(Writer&& w) : fd_(std::forward<boost::asio::posix::stream_descriptor>(w.fd_)) {}

  template<typename F>
  void operator()(F const& f) {
    boost::asio::async_write(fd_, const_buffers(f), [f](boost::system::error_code const&, size_t) {});
  }

  boost::asio::posix::stream_descriptor fd_;
};


}}

#endif
