#ifndef __VS_MEDIA_HPP__
#define __VS_MEDIA_HPP__

#include <queue>
#include <algorithm>
#include <numeric>
#include <boost/asio.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/bind/apply.hpp>
#include <boost/fusion/include/at_key.hpp>
#include <boost/tuple/tuple.hpp>

boost::int16_t alaw2linear(boost::uint8_t);
boost::uint8_t linear2alaw(boost::int16_t);
boost::int16_t ulaw2linear(boost::uint8_t);
boost::uint8_t linear2ulaw(boost::int16_t);

namespace Media {

extern boost::asio::io_service g_io;

namespace Transport {

struct UdpPacket {
  UdpPacket(boost::shared_ptr<void> const& p, size_t length) : data_(p), length_(length) {}

  boost::shared_ptr<void> data_;
  size_t length_;
};

struct Socket  {
  Socket() : socket_(boost::make_shared<boost::asio::ip::udp::socket>(g_io)) {}

  void operator()(UdpPacket const& p);

  template<typename S>
  void recv(S s) { recv(socket_, s); }

  template<typename S>
  static void recv(boost::shared_ptr<boost::asio::ip::udp::socket> const& socket, S s) {
    if(socket) {
      boost::shared_ptr<void> buffer = boost::make_shared<boost::array<boost::uint8_t, 1360> >();
      socket->async_receive(boost::asio::mutable_buffers_1(buffer.get(), 1360), boost::bind(on_recv<S>, socket, s, buffer, _1, _2));
    }
  }

  template<typename S>
  static void on_recv(boost::weak_ptr<boost::asio::ip::udp::socket> const& socket, S s, boost::shared_ptr<void> const& buffer,
    boost::system::error_code const& ec, size_t bytes)
  {
    if(!ec) {
      s(UdpPacket(buffer, bytes));
      recv(boost::shared_ptr<boost::asio::ip::udp::socket>(socket), s);
    }
  }

  boost::shared_ptr<boost::asio::ip::udp::socket> socket_;
};

struct RtpPacket {
  boost::uint8_t pt_;
  bool marker_; 
  boost::uint16_t seq_;
  boost::uint32_t ts_; 
  boost::uint32_t ssrc_;
  
  boost::shared_ptr<void> payload_;
  boost::uint32_t length_;
};

bool parse(UdpPacket const& in, RtpPacket& out);
bool parse(RtpPacket const& in, UdpPacket& out);

template<typename To, typename A>
struct Parse {
  Parse(A a) : a_(a) {}

  template<typename From>
  void operator()(From const& p) {
    To t;
    if(parse(p, t)) a_(t);
  }

  A a_;
};

template<typename To, typename A>
Parse<To, A> make_parse(A const& a) {
  return Parse<To, A>(a);
}

template<typename F, typename A>
struct RtpToPacket {
  RtpToPacket(boost::uint8_t pt, A a) : pt_(pt), a_(a) {}

  void operator()(RtpPacket const& p) {
    if(p.pt_ == pt_) {
      F f;
      if(parse(p, f))
        a_(f);
    }    
  }

  boost::uint8_t pt_;
  A a_;
};

template<typename F, typename A>
RtpToPacket<F, A> rtp2packet(boost::uint8_t pt, A const& a) {
  return RtpToPacket<F,A>(pt, a);
}

template<typename M, typename A>
struct PacketToRtp {
  PacketToRtp(M const& m, A const& a) : ssrc_(rand()), m_(m), a_(a) {}
  
  template<typename T>
  void operator()(T const& t) {
    RtpPacket p;
    p.ssrc_ = ssrc_;
    p.ts_ = t.ts_;
    p.seq_ = t.seq_;
    p.marker_ = t.marker_;

    if(-1 != boost::fusion::at_key<T>(m_)) {
      a_(p);
    }    
  }

  boost::uint32_t ssrc_;
  M m_;
  A a_;
};

template<typename F, typename S = boost::asio::posix::stream_descriptor>
struct Reader {
  template<typename... Args>
  Reader(Args... args) : data_(boost::make_shared<Data>(boost::ref(g_io), args...)) {}
 
  struct Data {
    template<typename... Args>
    Data(Args... args) : s_(args...) {}
    
    S s_;
    F f_;
  };
 
  F operator()() {
    boost::asio::async_read(data_->s_, mutable_buffers(data_->f_), boost::bind(read_handler, data_, _1, _2));
    return data_->f_;
  }

  static void read_handler(boost::shared_ptr<Data> const&, boost::system::error_code const& ec, size_t) {}

  boost::shared_ptr<Data> data_;
};

template<typename S = boost::asio::posix::stream_descriptor>
struct Writer {
  template<typename... Args>
  Writer(Args... args) : data_(boost::make_shared<S>(g_io, args...)) {}

  template<typename F>
  void operator()(F const& f) {
    boost::asio::async_write(*data_, const_buffers(f), boost::bind(write_handler<F>, f, _1, _2));
  }

  template<typename F>
  static void write_handler(F const&, boost::system::error_code const& ec, size_t) {}
  
  boost::shared_ptr<S> data_;
};

}

namespace Audio {

typedef boost::uint32_t Timestamp;

template<typename Frame>
struct Packet : Frame {
  Packet() {}
  Packet(Frame const& f, Timestamp ts) : Frame(f), ts_(ts) {}

  Timestamp ts_;
};


template<size_t N>
struct LinearFrame {
  static const size_t duration = N;
  boost::shared_ptr<boost::array<boost::int16_t, N> > data_;
};


template<size_t N>
LinearFrame<N> operator+ (LinearFrame<N> const& a, LinearFrame<N> const& b) {
  if(a.data_ && b.data_) {
    LinearFrame<N> f;
    f.data_ = boost::make_shared<boost::array<boost::int16_t,N>>();
    boost::int16_t *fd = f.data_->begin(), * ad = a.data_->begin(), * bd = b.data_->begin(); 
    for(size_t i = 0; i < N;++i)
      fd[i] = ad[i] + bd[i];
    return f;
  }
  else if(a.data_)
    return a;

  return b;
}

template<size_t N>
boost::asio::const_buffers_1 empty_frame_data(LinearFrame<N>) {
  static boost::uint16_t data[N] = {0};
  return boost::asio::const_buffers_1(data, sizeof(data));
} 

template<typename F>
boost::asio::const_buffers_1 const_buffers(F const& f) {
  if(f.data_)
    return boost::asio::const_buffers_1(f.data_->begin(), f.data_->size() * sizeof(*f.data_->begin()));
  return empty_frame_data(f);
}

template<typename F>
boost::asio::mutable_buffers_1 mutable_buffers(F& f) {
  return boost::asio::mutable_buffers_1(f.data_->begin(), f.data_->size() * sizeof(*f.data_->begin()));
}

template<size_t N, typename A>
struct Resizer {
  typedef decltype((*((A*)(0)))()) FromFrame;
  static const int M = FromFrame::duration;
  
  BOOST_STATIC_ASSERT((N % M == 0 || M % N == 0));

  template<size_t F, size_t T>
  struct Up {
    Up(A const& a) : a_(a) {}

    LinearFrame<T> operator()() {
      LinearFrame<T> f;
      f.data_ = boost::make_shared<boost::array<boost::int16_t, T> >();
      for(size_t i = 0; i < T/F; ++i) {
        LinearFrame<F> a = a_();
        if(a.data_)
          memcpy(f.data_.get() + i * F, a.data_.get(), sizeof(*a.data_.get()));
        else
          memset(f.data_.get() + i * F, 0, sizeof(*a.data_.get()));
      }
      
      return f;
    }
    A a_;
  };
  
  template<size_t F, size_t T>
  struct Down {
    Down(A const& a) : a_(a), state_(0) {}

    LinearFrame<T> operator()() {
      if(0 == state_) {
        f_ = a_();
      }

      LinearFrame<T> r;
      r.data_ = boost::shared_ptr<boost::array<boost::int16_t,T> >(f_.data_, (boost::array<boost::int16_t,T>*)(f_.data_.get() + state_ * T));

      state_ = (state_ + 1) % (F/T);
      return r;
    }
  
    A a_;
    LinearFrame<F> f_;
    size_t state_;
  };
  
  typename boost::mpl::if_c<(N > M), Up<M, N>, Down<M, N> >::type action_;

  Resizer(A const& a) : action_(a) {}

  LinearFrame<N> operator()() {
    return action_();
  } 
};

template<int N, typename A>
Resizer<N,A> resize(A const& a) {
  return Resizer<N,A>(a);
}

template<typename Frame, typename Source>
struct Decode;

template<typename A>
auto decode(A a) -> Decode<decltype(a()), A> {
  return Decode<decltype(a()),A>(a);
}

template<typename Frame, typename Source>
struct Encode;

template<typename F, typename A>
Encode<F, A> encode(A const& a) {
  return Encode<F, A>(a);
}

template<typename F>
bool parse(Transport::RtpPacket const& in, Packet<F>& out) {
  if(in.length_ == sizeof(*out.data_)) {
    typedef decltype(out.data_) Data;
    out.data_ = Data(in.payload_, reinterpret_cast<decltype(&*out.data_)>(in.payload_.get()));
    out.ts_ = in.ts_;
    return true;
  }
  return false;
}


template<size_t N>
struct AlawFrame {
  static const size_t duration = N;

  boost::shared_ptr<boost::array<boost::uint8_t, N> > data_;
};

template<size_t N>
boost::asio::const_buffers_1 empty_frame_data(AlawFrame<N>) {
  static boost::uint8_t data[0];
  if(data[0] != 0x55) std::fill(data, data+N, 0x55);
  return boost::asio::const_buffers_1(data,N);
}

template<size_t N, typename A>
struct Decode<AlawFrame<N>,A> {
  Decode(A const& a) : a_(a) {}

  LinearFrame<N> operator()() {
    AlawFrame<N> f = a_();
    LinearFrame<N> r;
    if(f.data_) {
      r.data_ = boost::make_shared<boost::array<boost::int16_t, N> >();
      std::transform(f.data_.get()->begin(), f.data_.get()->end(), r.data_->begin(), alaw2linear);
    }
    return r;
  }

  A a_;
};

template<size_t N, typename A>
struct Encode<AlawFrame<N>,A> {
  Encode(A const& a) : a_(a) {}

  AlawFrame<N> operator()() {
    LinearFrame<N> f = a_();
    AlawFrame<N> r;
    if(f.data_) {
      r.data_ = boost::make_shared<boost::array<boost::uint8_t, N> >();
      std::transform(f.data_.get()->begin(), f.data_.get()->end(), r.data_->begin(), linear2alaw);  
    }
    return r;
  }

  A a_;
};


template<size_t N>
struct UlawFrame { 
  static const size_t duration = N;

  boost::shared_ptr<boost::array<boost::uint8_t, N> > data_;
};

template<size_t N>
boost::asio::const_buffers_1 empty_frame_data(UlawFrame<N>) {
  static boost::uint8_t data[0];
  if(data[0] != 0x55) std::fill(data, data+N, 0xFF);
  return boost::asio::const_buffers_1(data,N);
}

template<size_t N, typename A>
struct Decode<UlawFrame<N>,A> {
  Decode(A const& a) : a_(a) {}

  LinearFrame<N> operator()() {
    UlawFrame<N> f = a_();
    LinearFrame<N> r;
    if(f.data_) {
      r.data_ = boost::make_shared<boost::array<boost::int16_t, N> >();
      std::transform(f.data_.get()->begin(), f.data_.get()->end(), r.data_->begin(), ulaw2linear);
    }
    return r;
  }

  A a_;
};

template<size_t N, typename A>
struct Encode<UlawFrame<N>,A> {
  Encode(A const& a) : a_(a) {}

  UlawFrame<N> operator()() {
    LinearFrame<N> f = a_();
    UlawFrame<N> r;
    if(f.data_) {
      r.data_ = boost::make_shared<boost::array<boost::uint8_t, N> >();
      std::transform(f.data_.get()->begin(), f.data_.get()->end(), r.data_->begin(), linear2ulaw);  
    }
    return r;
  }

  A a_;
};


template<size_t N>
struct IlbcFrame;

template<>
struct IlbcFrame<20*8> {
  static const size_t duration = 20*8;
  boost::shared_ptr<boost::array<boost::uint8_t, 28> > data_;
};

template<>
struct IlbcFrame<30*8> {
  static const size_t duration = 30*8;
  boost::shared_ptr<boost::array<boost::uint8_t, 50> > data_;
};

template<size_t N>
boost::asio::const_buffers_1 empty_frame_data(IlbcFrame<N>) {
  static boost::uint8_t data[N] = {0};
  data[N-1] = 0x80;
  return boost::asio::const_buffers_1(data, N);
}

namespace Detail {
struct IlbcDecoder {
  IlbcDecoder(int mode);

  void operator()(boost::uint8_t const* in, boost::int16_t* out);

  boost::shared_ptr<void> state_;
};

struct IlbcEncoder {
  IlbcEncoder(int mode);

  void operator()(boost::int16_t const* in, boost::uint8_t* out);

  boost::shared_ptr<void> state_;
};
}

template<size_t N, typename A>
struct Decode<IlbcFrame<N>, A> {
  Decode(A const& a) : a_(a), dec_(N/8) {}
  
  LinearFrame<N> operator()() {
    LinearFrame<N> r;
    r.data_ = boost::make_shared<boost::array<boost::int16_t, LinearFrame<N>::duration> >();
    dec_(IlbcFrame<N>(a_()).data_->begin(), r.data_->begin());
    return r;
  }
  
  A a_;
  Detail::IlbcDecoder dec_;
};

template<size_t N, typename A>
struct Encode<IlbcFrame<N>, A> {
  Encode(A const& a) : a_(a), enc_(N/8) {}

  IlbcFrame<N> operator()() {
    IlbcFrame<N> r;
    r.data_ = boost::make_shared<boost::array<boost::uint8_t, sizeof(*r.data_)>>();
    enc_(LinearFrame<N>(a_()).data_->begin(), r.data_->begin());
    return r;
  }

  A a_;
  Detail::IlbcEncoder enc_;
};


template<typename Frame>
struct JitterBuffer {
  JitterBuffer() : current_(0) {}

  void push(Packet<Frame> const& packet) {
    int n = (packet.ts_ - current_) / Frame::duration;

    if(abs(n) > 5) { // resync
      n = 2; 
      current_ = packet.ts_ - Frame::duration * n;
      frames_.clear();
    }

    if(frames_.size() < (n + 1)) frames_.resize(n+1); 
    frames_[n] = packet;
  }

  Frame pull() {
    Frame f;

    current_ += Frame::duration;

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


template<typename A, size_t N = 80>
struct Mixer {
  Mixer(A const& a) : a_(a) {}

  LinearFrame<N> operator()() {
    return std::accumulate(a_.begin(), a_.end(), LinearFrame<80>(), boost::bind(std::plus<LinearFrame<N>>(), _1, boost::bind(boost::apply<LinearFrame<N>>(),_2)));
  }

  A a_;
};


template<typename S>
struct Splitter {
  Splitter(S const& s) : s_(s) {}

  template<typename T>
  void operator()(T const& t) {
    for_each(s_.begin(), s_.end(), boost::bind(boost::apply<void>(), _1, boost::ref(t)));
  }

  S s_;
};

template<typename S>
struct PullBranch {
  typedef decltype(S()(0).first) D;

  PullBranch(S s) : data_(boost::make_shared<std::pair<S, size_t>>(std::make_pair(s, 0))) {}

  D operator()() {
    D d;
    boost::tie(d, data_->second) = data_->first(data_->second+1);
    return d;
  }

  boost::shared_ptr<std::pair<S, size_t>> data_;
};

template<typename A>
struct PullSplitter {
  typedef decltype((*(A*)0)()) D;

  PullSplitter(A const& a) : a_(a), n_(0) {}

  std::pair<D,size_t> operator()(size_t i) {
    assert(n_ == i || (n_+1) == i);

    if((n_+1)==i) {
      d_ = a_();
      ++n_;
    }

    return std::make_pair(d_, n_);
  }

  PullBranch<decltype(boost::bind<std::pair<D, size_t>>((PullSplitter*)0, _1))> branch() { return boost::bind(this, _1); }

  A a_;
  D d_;
  size_t n_;
};


template<typename Source, typename Sink>
struct Clock {
  Clock(Source const& src, Sink const& sk) : timer_(boost::make_shared<boost::asio::deadline_timer>(g_io)) {
    schedule(timer_, src, sk, boost::posix_time::microsec_clock::universal_time());
  }
private:
  typedef decltype ((*((Source*)(0)))()) SourceResult;
  static const int period = SourceResult::duration;

  static void schedule(boost::shared_ptr<boost::asio::deadline_timer> const& timer, Source const& src, Sink const& sk, boost::posix_time::ptime ts) {
    ts += boost::posix_time::microseconds(125)*period;
    if(timer) {
      timer->expires_at(ts);
      timer->async_wait(boost::bind(on_timer, timer, src, sk, ts, _1));
    }
  }

  static void on_timer(boost::weak_ptr<boost::asio::deadline_timer> const& timer, Source& src, Sink& sk,
    boost::posix_time::ptime const& ts,  boost::system::error_code const& ec)
  {
    if(!ec) {
      schedule(boost::shared_ptr<boost::asio::deadline_timer>(timer), src, sk, ts);
      sk(src());
    }
  }

  boost::shared_ptr<boost::asio::deadline_timer> timer_;
};

template<typename Source,typename Sink>
Clock<Source, Sink> clock(Source const& src, Sink const& sk) {
  return Clock<Source, Sink>(Source(src), Sink(sk));
}


template<typename A>
struct Packetizer {
  Packetizer(A a) : a_(a) {}

  template<typename T>
  void operator()(T const&  t) {
    a_(Packet<T>(t, ts_));
    ts_ += T::duration;
  }
  
  A a_;
  boost::uint32_t ts_;
};

}

void start();
void stop();

template<typename F>
void post(F f) {
  g_io.post(f);
}

}

#endif
