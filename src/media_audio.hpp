#ifndef __MEDIA_AUDIO_HPP__
#define __MEDIA_AUDIO_HPP__

#include "media_core.hpp"
#include "media_transport.hpp"
#include <deque>
#include <boost/bind/apply.hpp>
 
std::int16_t alaw2linear(std::uint8_t);
std::int16_t ulaw2linear(std::uint8_t);
std::uint8_t linear2alaw(std::int16_t);
std::uint8_t linear2ulaw(std::int16_t);

namespace Media{namespace Audio {
typedef boost::uint32_t Timestamp;

template<typename Frame>
struct Packet {
  Packet() {}
  Packet(Frame const& f, Timestamp ts): f_(f), ts_(ts) {}

  Frame f_;
  Timestamp ts_;
};

template<typename P>
struct FrameType;

template<typename F>
struct FrameType<Packet<F> > {
  typedef F type;
};

template<template<size_t N> class F, size_t N>
constexpr size_t duration(F<N> const& f) {
  return N;
}

template<size_t N, size_t M, template<size_t> class F>
F<N> resize_type(F<M> const&);


template<typename F>
Timestamp transform(F const& f, std::pair<bool,Packet<F>>& p, Timestamp s = rand()) {
  p.first= false;
  if(f.d_)
    p = std::make_pair(true,Packet<F>(f, s));

  return s + duration(f);
}

template<typename F>
int transform(Transport::RtpPacket const& f, std::pair<bool, Packet<F>>& t, int pt = 0) {
  t.first = false;
  if(f.hdr_->pt == pt && f.length_ == sizeof(*t.second.f_.d_)) {
    t.second.ts_ = f.hdr_->ts;
    typedef decltype(t.second.f_.d_) D;
    t.second.f_.d_ = D(f.data_, static_cast<typename D::element_type*>(f.data_.get()));
    t.first = true;
  }
  return pt;
}

struct RtpFyState {
  RtpFyState(std::uint8_t pt, std::uint32_t ssrc = rand(), std::uint32_t seq = rand()) : pt_(pt), ssrc_(ssrc), seq_(seq) {}

  std::uint8_t pt_;
  std::uint32_t ssrc_;
  std::uint32_t seq_;
};

template<typename F>
RtpFyState&& transform(Packet<F> const& f, Transport::RtpPacket& t, RtpFyState&& s = RtpFyState(0)) {
  t.hdr_ = std::make_shared<Transport::RtpHeader>();
  memset(t.hdr_.get(), 0, sizeof(*t.hdr_));

  t.hdr_->version = 2;
  t.hdr_->ssrc = s.ssrc_;
  t.hdr_->pt = s.pt_;
  t.hdr_->seq = s.seq_;

  t.data_ = f.f_.d_;
  
  return std::move(s);
}

template<typename F>
boost::asio::const_buffers_1 const_buffers(F const& f) {
  if(f.d_)
    return boost::asio::const_buffers_1(&(*f.d_), sizeof(*f.d_));
  return boost::asio::const_buffers_1(0,0);
}

template<typename Frame>
boost::asio::mutable_buffers_1 mutable_buffers(Frame& f) {
  if(!f.d_)
    f.d_ = std::make_shared<typename std::decay<decltype(*f.d_)>::type>();
  return boost::asio::mutable_buffers_1(&(*f.d_), sizeof(*f.d_));
}


template<size_t N>
struct LinearFrame {
  std::shared_ptr<std::array<std::int16_t, N>> d_;
};

template<size_t N, size_t M>
void transform(LinearFrame<N> const& f, std::array<LinearFrame<N/M>, M>& t) {
  for(size_t i = 0; i != M; ++i)
    t[i].d_ = std::shared_ptr<std::array<std::int16_t, N/M>>(f.d_, reinterpret_cast<std::array<std::int16_t, N/M>*>(&(*f.d_)[i*N/M]));
}

template<size_t N, size_t M>
void transform(std::array<LinearFrame<N/M>, M> const& f, LinearFrame<N>& t) {
  mutable_buffers(t);
  for(size_t i = 0; i != M; ++i)
    memcpy(t.d_->begin() + i * N/M, f[i].d_->begin(), sizeof(*f[i].d_));
}

template<size_t N>
LinearFrame<N>& operator += (LinearFrame<N>& a, LinearFrame<N> const& b) {
  if(b.d_) {
    mutable_buffers(a);
    for(size_t i = 0; i != N; ++i)
      (*a.d_)[i] += (*b.d_)[i];
  }
  return a;
}

template<size_t N>
struct AlawFrame {
  static const size_t duration = N;
  std::shared_ptr<std::array<boost::uint8_t, N>> d_;
};

template<size_t N>
void transform(AlawFrame<N> const& f, LinearFrame<N>& t) {
  if(f.d_) {
    mutable_buffers(t);
    for(size_t i = 0; i != N; ++i)
      (*t.d_)[i] = alaw2linear((*f.d_)[i]);
  }
}

template<size_t N>
void transform(LinearFrame<N> const& f, AlawFrame<N>& t) {
  if(f.d_) {
    mutable_buffers(t);
    for(size_t i = 0; i != N; ++i)
      (*t.d_)[i] = linear2alaw((*f.d_)[i]);
  }
}


template<size_t N>
struct UlawFrame {
  std::shared_ptr<std::array<boost::uint8_t, N>> d_;
};

template<size_t N>
void transform(UlawFrame<N> const& f, LinearFrame<N>& t) {
  if(f.d_) {
    mutable_buffers(t);
    for(size_t i = 0; i != N; ++i)
      (*t.d_)[i] = ulaw2linear((*f.d_)[i]);
  }
}

template<size_t N>
void transform(LinearFrame<N> const& f, UlawFrame<N>& t) {
  if(f.d_) {
    mutable_buffers(t);
    for(size_t i = 0; i != N; ++i)
      (*t.d_)[i] = linear2ulaw((*f.d_)[i]);
  }
}


template<size_t N>
struct IlbcFrame;

template<>
struct IlbcFrame<8*20> {
  std::shared_ptr<std::array<std::uint8_t, 28>> d_;
};

template<>
struct IlbcFrame<8*30> {
  std::shared_ptr<std::array<std::uint8_t, 50>> d_;
};

template<size_t N>
struct IlbcDecState {
  IlbcDecState() {} 
  char data_[128];
};

template<size_t N>
struct IlbcEncState {
  IlbcEncState() {}
  char data_[128];
};

template<size_t N>
IlbcDecState<N>& transform(IlbcFrame<N> const&, LinearFrame<N>&, IlbcDecState<N>&& = IlbcDecState<N>());

template<size_t N>
IlbcEncState<N>& transform(LinearFrame<N> const&, IlbcFrame<N>&, IlbcEncState<N>&& = IlbcEncState<N>());


template<typename A, typename F = LinearFrame<duration(*(typename SourceType<A>::type*)(0))>>
auto decode(A&& a) -> decltype(transform_to<F>(std::forward<A>(a))) {
  return transform_to<F>(std::forward<A>(a));
}

template<typename F, typename A>
auto encode(A&& a) -> decltype(transform_to<F>(std::forward<A>(a))) {
  return transform_to<F>(std::forward<A>(a));
}

template<size_t N, typename A, typename F = typename SourceType<A>::type, size_t M = duration(*(F*)0), typename R = decltype(resize_type<N>(*(F*)0))>
auto resize(A&& a, typename std::enable_if<M >= N>::type* =0) -> decltype(disassemble(transform_to<std::array<R,M/N>>(std::forward<A>(a)))) {
  return disassemble(transform_to<std::array<R,M/N>>(std::forward<A>(a)));
}

template<size_t N, typename A, typename F = typename SourceType<A>::type, size_t M = duration(*(F*)0), typename R = decltype(resize_type<N>(*(F*)0))>
auto resize(A&& a, typename std::enable_if<M < N>::type* = 0) -> decltype(transform_to<R>(assemble<N/M>(std::forward<A>(a)))) {
  return transform_to<R>(assemble<N/M>(std::forward<A>(a)));
}

template<typename A, size_t N = 80>
struct Mixer {
  Mixer(A && a) : a_(std::forward<A>(a)) {}
 
  template<typename C>
  void operator()(C c) {
    LinearFrame<N> r;
    i_ = 0;
     
    for(auto& a: a_)
      a([=,this](LinearFrame<N> const& f) mutable {
        r += f;
        if(++i_ == a_.size()) c(r);
      });
  }

  A a_;
  size_t i_;
};

template<typename A>
Mixer<A> mix(A&& a) {
  return Mixer<A>(std::forward<A>(a));
}

template<typename Frame>
struct JitterBuffer {
  JitterBuffer() : current_(0) {}
  JitterBuffer(JitterBuffer&& r) : frames_(std::forward<Frames>(r.frames_)), current_(r.current_) {}

  void operator()(Packet<Frame> const& packet) {
    int n = (packet.ts_ - current_) / duration(Frame());

    if(abs(n) > 5) { // resync
      n = 2;
      current_ = packet.ts_ - duration(Frame()) * n;
      frames_.clear();
    }

    if(n >= 0) {
      if(frames_.size() < size_t(n + 1)) frames_.resize(n+1);
      frames_[n] = packet.f_;
    }
  }

  Frame operator()() {
    Frame f;

    current_ += duration(f);

    if(!frames_.empty()) {
      f = frames_.front();
      frames_.pop_front();
    }  

    return f;
  }

  typedef std::deque<Frame> Frames;
  Frames frames_;
  Timestamp current_;
};

template<typename A, typename F = typename FrameType<typename SourceType<A>::type>::type>
auto jitter(A&& a) -> decltype(pull(push(std::forward<A>(a), JitterBuffer<F>()))) {
  return pull(push(std::forward<A>(a), JitterBuffer<F>()));
}

template<size_t N>
struct Tick {
  Tick() : ts_(boost::posix_time::microsec_clock::universal_time()), timer_(g_io) {
    start();
  }

  void start() {
    ts_ +=  boost::posix_time::microseconds(125) * N;
    timer_.expires_at(ts_);
    (*this)(std::bind(&Tick<N>::start, this));
  }

  template<typename C>
  void operator()(C c) {
    typedef decltype(c()) x;
    timer_.async_wait([=](boost::system::error_code const& e) { if(e) c();});
  }

  boost::posix_time::ptime ts_;
  boost::asio::deadline_timer timer_;

  static Tick<N>& get() {
    static Tick<N> t;
    return t;
  }
};

template<size_t N, typename C>
void next_tick(C&& c) {
  Tick<N>::get()(std::forward<C>(c));
}

template<typename A>
struct Rate {
  typedef typename SourceType<A>::type source_type;
  Rate(A&& a) : a_(std::make_shared<A>(std::forward<A>(a))) {}

  template<typename C>
  void operator()(C c) {
    std::weak_ptr<A> a(a_);
    next_tick<duration(*(source_type*)(0))>([=] {
      auto b = a.lock();
      if(b)
        (*b)(c);
      }
    );
  }

  std::shared_ptr<A> a_;
};

template<typename A>
Rate<A> rate(A&& a) {
  return Rate<A>(std::forward<A>(a));
}

}}

#endif
