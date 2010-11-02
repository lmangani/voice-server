#ifndef __S3_MEDIA_HPP__
#define __S3_MEDIA_HPP__

#include <iostream>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/intrusive_ptr.hpp>
#include <boost/detail/atomic_count.hpp>

//Simpliest possible audio processing library for ip-telephony
namespace Media {

template<typename T>
struct Refcounted : boost::noncopyable {
  Refcounted() : refs_(0) {}
private:
  template<typename T2>
  friend void intrusive_ptr_add_ref(Refcounted<T2>*);

  template<typename T2>
  friend void intrusive_ptr_release(Refcounted<T2>*);
  boost::detail::atomic_count refs_;
};

template<typename T>
void intrusive_ptr_add_ref(Refcounted<T>* t) {
  ++t->refs_;
}

template<typename T>
void intrusive_ptr_release(Refcounted<T>* t) {
  if((--t->refs_)==0)
    delete static_cast<T*>(t);
}

namespace asio = boost::asio;

extern asio::io_service g_io;

namespace posix_time = boost::posix_time;

#ifdef DEBUG
struct TimerGuard {
  TimerGuard() : ts_(posix_time::microsec_clock::universal_time()) {}
  ~TimerGuard() {
    posix_time::time_duration d = posix_time::microsec_clock::universal_time() - ts_;
    if(d > posix_time::milliseconds(10)) {
      std::cerr << "\n\n\n" << d << "\n\n\n";
      assert(0);
    }
  }

  posix_time::ptime ts_;
};
#else
struct TimerGuard {};
#endif

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

struct PayloadTraits : boost::noncopyable {
  virtual ~PayloadTraits() {}
  virtual const char* name() const = 0;
  virtual posix_time::time_duration frame_duration() const = 0;
  virtual size_t frame_size() const = 0;
  
  virtual posix_time::time_duration default_packet_duration() const = 0; 
  
  virtual posix_time::time_duration rtp_timestamp_to_duration(boost::uint32_t ts) const = 0;
  virtual boost::uint32_t duration_to_rtp_timestamp(posix_time::time_duration const& duration) const = 0;

  virtual void fill_silence(void* data, size_t size) const =0;
};

typedef PayloadTraits const* PayloadType;

inline
size_t duration_to_size(PayloadType pt, posix_time::time_duration const& d) {
  return d.total_microseconds() / pt->frame_duration().total_microseconds() * pt->frame_size();
}

inline
posix_time::time_duration size_to_duration(PayloadType pt, size_t s) {
  return pt->frame_duration() * (s / pt->frame_size());
} 
 
inline
posix_time::time_duration default_packet_duration(PayloadType pt) {
  return pt->default_packet_duration();
}

inline
boost::uint32_t duration_to_rtp_timestamp(PayloadType pt, posix_time::time_duration const& d) {
  return pt->duration_to_rtp_timestamp(d);
}

inline
posix_time::time_duration rtp_timestamp_to_duration(PayloadType pt, boost::uint32_t t) {
  return pt->rtp_timestamp_to_duration(t);
}

PayloadTraits const* get_g711a_traits();
PayloadTraits const* get_g711u_traits();
PayloadTraits const* get_rfc2833_traits();
PayloadTraits const* get_l16_traits();

static const PayloadType G711A = get_g711a_traits();
static const PayloadType G711U = get_g711u_traits();
static const PayloadType RFC2833 = get_rfc2833_traits();
static const PayloadType L16 = get_l16_traits();

static const PayloadType Payloads[] = {G711A, G711U, RFC2833, L16};

struct Packet : boost::noncopyable {
  posix_time::ptime const& timestamp() const { return timestamp_; }
  posix_time::time_duration const& duration() const { return duration_; }

  PayloadType payload_type() const { return payload_type_; }

  boost::uint8_t* samples() { return reinterpret_cast<boost::uint8_t*>(this)+sizeof(*this); }
  size_t size() const { return size_; }

 
  posix_time::ptime timestamp_;
  posix_time::time_duration duration_;

  PayloadType payload_type_;

  size_t size_;

private:
  Packet(size_t size) : size_(size), refs_(0) {}

  boost::detail::atomic_count refs_;
  
  friend 
  boost::intrusive_ptr<Packet> make_packet(size_t);

  friend
  void intrusive_ptr_add_ref(Packet*);

  friend
  void intrusive_ptr_release(Packet*);
};

inline
boost::intrusive_ptr<Packet> make_packet(size_t size) {
  size_t alloc = size + sizeof(Packet);

  void* d = new char[alloc];
  return new(d) Packet(size);
}

inline
void intrusive_ptr_add_ref(Packet* packet) {
  ++packet->refs_;
}

inline
void intrusive_ptr_release(Packet* packet) {
  if(--(packet->refs_) == 0) {
    packet->~Packet();
    delete[] reinterpret_cast<char*>(packet);
  }
}

typedef boost::intrusive_ptr<Packet> PacketPtr;

inline
PacketPtr make_packet(PayloadType pt, posix_time::ptime const& ts, posix_time::time_duration const& duration) {
  PacketPtr p = make_packet(duration_to_size(pt, duration));
  p->payload_type_ = pt;
  p->timestamp_ = ts;
  p->duration_ = duration;
  return p;
}

inline
PacketPtr make_packet(PayloadType pt, posix_time::ptime const& ts) {
  return make_packet(pt, ts, pt->default_packet_duration());
}

inline
boost::intrusive_ptr<Packet> make_silence_packet(PayloadType pt, posix_time::time_duration const& duration)
{
  boost::intrusive_ptr<Packet> packet = make_packet(duration_to_size(pt, duration));

  packet->duration_ = duration;

  packet->payload_type_ = pt;

  pt->fill_silence(packet->samples(), packet->size());

  return packet;
}

inline
boost::intrusive_ptr<Packet> make_silence_packet(PayloadType pt) {
  return make_silence_packet(pt, default_packet_duration(pt));
}

inline 
PacketPtr make_silence_packet(PayloadType pt, posix_time::ptime const& ts) {
  PacketPtr p = make_silence_packet(pt);

  p->timestamp_ = ts;

  return p;
}

namespace Detail {
  struct SinkVTable : Refcounted<SinkVTable> {
    virtual ~SinkVTable() {}

    virtual void operator()(PacketPtr const& packet) =0;
  };
}

struct Sink {
  Sink() {}
  Sink(boost::intrusive_ptr<Detail::SinkVTable> const& v) : v_(v) {}

  void operator()(PacketPtr const& packet) const {
    (*v_)(packet);
  }

  bool empty() const {
    return !v_;
  }

  bool operator ! () const {
    return !v_;
  }

  boost::intrusive_ptr<Detail::SinkVTable> v_;
};

template<typename Functor>
inline
Sink make_sink(Functor const& f) {
  using namespace Detail;

  struct VTable : SinkVTable {
    VTable(Functor const& f) : f_(f) {}

    void operator()(PacketPtr const& packet) {
      f_(packet);
    }
    Functor f_;
  };

  return Sink(new VTable(f));
}

namespace Detail {
  struct SourceVTable : Refcounted<SourceVTable> {
    virtual ~SourceVTable() {}
    virtual void operator()(Sink const& sink) =0;
  };
}

struct Source {
  Source() {}
  Source(boost::intrusive_ptr<Detail::SourceVTable> const& v) : v_(v) {}

  void operator()(Sink const& sink) {
    (*v_)(sink);
  }

  bool empty() const { return !v_; }

  bool operator!() const { return !v_; }

  boost::intrusive_ptr<Detail::SourceVTable> v_;
};

template<typename Functor>
inline
Source make_source(Functor const& f) {
  using namespace Detail;

  struct VTable : SourceVTable {
    VTable(Functor const& f) : f_(f) {}

    void operator()(Sink const& sink) {
      f_(sink);
    }
    Functor f_;
  };

  return Source(new VTable(f));
}

Source mix2(Source a, Source b);
Sink split2(Sink a, Sink b);

typedef boost::function<void (Source const&)> PullSink;
typedef boost::function<void (Sink const&)> PushSource;

template<typename Decoder>
inline
Source decoder(Source source);

template<typename Decoder>
Sink decoder(Sink);

template<typename Encoder>
Source encoder(Source);

template<typename Encoder>
Sink encoder(Sink);

std::pair<Source, Sink> make_jitter_buffer(PayloadType pt, posix_time::time_duration const& latency = posix_time::milliseconds(50));

struct Rtp;

//Sink make_sink(boost::intrusive_ptr<Rtp> rtp);

//void set_sink(boost::intrusive_ptr<Rtp> rtp, Sink sink);
//void set_source(boost::intrusive_ptr<Rtp> rtp, Source source);

struct FileSource;
//Source make_source(boost::intrusive_ptr<FileSource>);

struct FileSink;
//void set_source(boost::intrusive_ptr<FileSink>, Source);
//Sink make_sink(boost::intrusive_ptr<FileSink>);

// API implementation details
namespace Detail {
  template<typename T>
  struct FilterSink : public Detail::SinkVTable {
    FilterSink(T const& t, Sink sink) : t_(t), sink_(sink) {}

    void operator()(PacketPtr const& packet) {
      sink_(t_(packet));
    }

    T t_;
    Sink sink_;
  };

  template<typename T>
  Sink make_filter_sink(T const& t, Sink sink) {
    return Sink(new FilterSink<T>(t, sink));
  }

  template<typename T>
  struct FilterSource : SourceVTable {
    FilterSource(T const& t , Source source) :
      source_(source), sink_(make_filter_sink(t, Sink()))
    {}

    void operator()(Sink const& sink) {
      remember_sink(sink);
      source_(sink_);
    }

    void remember_sink(Sink sink) {
      static_cast<FilterSink<T>*>(sink_.v_.get())->sink_ = sink;
    }
 
    Source source_;
    Sink sink_;
  };

  template<typename Filter>
  inline
  Source make_filter_source(Filter f, Source source) {
    return Source(new FilterSource<Filter>(f, source));
  }

  struct Puller : SinkVTable {
    Puller(Source const& src, Sink const& sk) : timer_(g_io), source_(src), sink_(sk) {}
      
    void operator()(PacketPtr const& p) {
      TimerGuard tg;
      if(!!sink_) {
        ts_ += p->duration();
        timer_.expires_at(ts_);
        timer_.async_wait(boost::bind(&Puller::pull, boost::intrusive_ptr<Puller>(this), _1));
        {
          TimerGuard tg2;
          sink_(p);
        }
      }
    }

    void pull(boost::system::error_code const& ec) {
      TimerGuard tg;

      //std::cerr << posix_time::microsec_clock::universal_time() 
      //          << " puller " << this << " " << posix_time::microsec_clock::universal_time() - ts_ << " " << timer_.expires_at() << std::endl;
      if(!ec && !!source_) {
        source_(Sink(this));
      }
    }

    void start() {
      ts_ = posix_time::microsec_clock::universal_time();
      source_(Sink(this));
    }

    void stop() {
      timer_.cancel();
      source_ = Source();
      sink_ = Sink();
    }
  
    asio::deadline_timer timer_;
    posix_time::ptime ts_;
    Source source_;
    Sink sink_;
  };
}

struct G711aDecoder {
  PacketPtr operator()(PacketPtr const& packet) const;
};

struct G711aEncoder {
  PacketPtr operator()(PacketPtr const& packet) const;
};

struct G711uDecoder {
  PacketPtr operator()(PacketPtr const& packet) const;
};

struct G711uEncoder {
  PacketPtr operator()(PacketPtr const& packet) const;
};

template<typename Decoder>
inline
Source decoder(Source source) {
  return Detail::make_filter_source(Decoder(), source);
}

template<typename Decoder>
inline
Sink decoder(Sink sink) {
  return Detail::make_filter_sink(Decoder(), sink);
}

template<typename Encoder>
inline
Source encoder(Source source) {
  return Detail::make_filter_source(Encoder(), source);
}

template<typename Encoder>
inline
Sink encoder(Sink sink) {
  return Detail::make_filter_sink(Encoder(), sink);
}

struct Rtp : Refcounted<Rtp> {
  Rtp();

  void set_sink(Sink const& sink);
  void set_source(Source const& source);

  void push(PacketPtr const& p);
  
  void connect(asio::ip::udp::endpoint const& ep);
  void bind(asio::ip::udp::endpoint const& ep);

  void set_payload_type_to_rtp_type(boost::function<int (PayloadType)> const& f) {
    smap_ = f;
  }

  void set_rtp_type_to_payload_type(boost::function<PayloadType (int)> const& f) {
    rmap_ = f;
  }

  asio::ip::udp::endpoint local_endpoint();

  void close(); 
private:
  void on_packet_recv(PacketPtr const& packet, boost::system::error_code const& ec, size_t bytes);
  void enqueue_recv();

  asio::ip::udp::socket socket_;
  
  RtpHeader shdr_;
  posix_time::ptime sts_;
  boost::function<int (PayloadType)> smap_;

  RtpHeader rhdr_;
  posix_time::ptime rts_;
  boost::uint32_t rssrc_;
  boost::function<PayloadType (int)> rmap_;

  Sink sink_;
  boost::intrusive_ptr<Detail::Puller> puller_;
};

struct FileSource : Refcounted<FileSource> {
  FileSource(const char* name, PayloadType pt, boost::function<void ()> const& eof = boost::function<void ()>());

  void pull(Sink const& sink);

  void close();

  void set_sink(Sink const& sink);
//private:
  void on_read_complete(Sink const& sink, PacketPtr const&, boost::system::error_code const&, size_t);

#ifdef _WIN32_WINNT
  asio::windows::random_access_handle file_;
#else
  asio::posix::stream_descriptor file_;
#endif
  PayloadType pt_;
  boost::function<void ()> on_eof_;
  posix_time::ptime ts_;

#ifdef _WIN32_WINNT
  posix_time::ptime begin_ts_;
#endif

  boost::intrusive_ptr<Detail::Puller> puller_;
};

struct FileSink : Refcounted<FileSink> {
  FileSink(const char* name);

  void push(PacketPtr const& packet);

  void close();

  void set_source(Source const& src);
//private:
#ifdef _WIN32_WINNT
  asio::windows::random_access_handle file_;
  posix_time::ptime begin_ts_;
#else
  asio::posix::stream_descriptor file_;
#endif
  boost::intrusive_ptr<Detail::Puller> puller_;
};

struct TelephoneEventDetector : Refcounted<TelephoneEventDetector> {
  typedef boost::function<void (int, posix_time::ptime const&)> Callback;
  
  TelephoneEventDetector(Callback const& on_end, Callback const& on_begin = Callback()) 
    : on_begin_(on_begin), on_end_(on_end), in_event_(false), timer_(g_io) {}

  void push(PacketPtr const& packet);
private:
  void on_timeout(boost::system::error_code const& ec);

  boost::function<void (int event, posix_time::ptime const& ts)> on_begin_;
  boost::function<void (int event, posix_time::ptime const& ts)> on_end_;
  
  bool in_event_;
  boost::uint8_t event_;
  posix_time::ptime ts_;  
  asio::deadline_timer timer_;
};

struct TelephoneEventGenerator : Refcounted<TelephoneEventGenerator> {
  void set_sink();
};

template<typename T>
inline
void set_source(boost::intrusive_ptr<T> sk, Source src) {
  g_io.post(boost::bind(&T::set_source, sk, src));
}

template<typename T>
inline
void set_sink(boost::intrusive_ptr<T> src, Sink sk) {
  g_io.post(boost::bind(&T::set_sink, src, sk));
}

template<typename T>
inline
Source make_source(boost::intrusive_ptr<T> p) {
  return make_source(boost::bind(&T::pull, p, _1));
}

template<typename T>
inline
Sink make_sink(boost::intrusive_ptr<T> p) {
  return make_sink(boost::bind(&T::push, p, _1));
}


void start();
}

#endif
