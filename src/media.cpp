#include "media.hpp"
#include <boost/detail/atomic_count.hpp>
#include <boost/utility.hpp>
#include <boost/bind.hpp>
#include <set>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

extern "C" {
#include "../ilbc/iLBC_decode.h"
#include "../ilbc/iLBC_encode.h"
}

#ifdef __MACH__
#include <mach/mach_init.h>
#include <mach/thread_policy.h>
#include <mach/thread_act.h>
#include <mach/mach_time.h>
#endif

short alaw2linear(unsigned char);
short ulaw2linear(unsigned char);
unsigned char linear2ulaw(short);
unsigned char linear2alaw(short);

namespace Media {

namespace asio = boost::asio;

asio::io_service g_io;

template<typename T>
struct RefcountedT {
  RefcountedT() : refs_(0) {}
  
  boost::detail::atomic_count refs_;
};

template<typename T>
void intrusive_ptr_add_ref(RefcountedT<T>* t) {
  ++t->refs_;
}

template<typename T>
void intrusive_ptr_release(RefcountedT<T>* t) {
  if(!(--t->refs_))
    delete static_cast<T*>(t);
}

posix_time::time_duration size_to_duration(Encoding enc, size_t size) {
  switch(enc) {
  case ilbc:
    return posix_time::milliseconds(38 == size ? 20 : 30);
  case linear:
    return posix_time::microseconds(125)*(size/2);
  default:
    return posix_time::microseconds(125)*size;
  }
}


struct Payload {
  Payload(size_t size) : refs_(0), size_(size) {}
  boost::detail::atomic_count refs_;
  size_t size_;
  char data_[0];
};

boost::intrusive_ptr<Payload> make_payload(size_t size) {
  char* data = new char[size + sizeof(Payload)]; 
  return new(data) Payload(size);
}

void intrusive_ptr_release(Payload* p) {
  if(!(--p->refs_)) delete[] p;
}

void intrusive_ptr_add_ref(Payload* p) {
  ++p->refs_;
}

struct Mixer : RefcountedT<Mixer> {
  Mixer(Sink const& s) : sink_(s) {}

  void push(Packet p) {
    if(p.encoding_ != linear)
      return;
    
    if(packets_.empty()) {
      packets_.push_back(p);
    }
    else if(p.srcid_ == packets_.front().srcid_) {
      if(ts_.is_special()) 
        ts_ = posix_time::microsec_clock::universal_time();
      else
        ts_ += packets_.front().duration_;

      Packet mixed(this, linear, ts_, packets_.front().duration_);

      memset(mixed.payload_->data_, 0, mixed.payload_->size_);

      for(Packets::iterator i = packets_.begin(), e = packets_.end(); i != e; ++i) {
        unsigned short *p = (unsigned short*)mixed.payload_->data_, 
          *q = (unsigned short*)(i->payload_->data_),
          *pe = p + mixed.payload_->size_ / sizeof(*p);
          if(q) for(; p != pe; ++p, ++q) *p += *q;
      }

      packets_.clear();

      sink_(mixed);
    }
    else {
      if(p.duration_ != packets_.front().duration_) 
        throw std::runtime_error("invalid packet length");
      packets_.push_back(p);
    }
  }
  
  posix_time::ptime ts_;
  typedef std::list<Packet> Packets;
  Packets packets_;
  Sink sink_;
};

Sink mixer(Sink const& s) {
  return boost::bind(&Mixer::push, boost::intrusive_ptr<Mixer>(new Mixer(s)), _1);
}


struct Encoders {
  Source get(Encoding e) {
    if(!sources_[e]) {
      Filter f = factories[e]();
      sources_[e] = f.first;
      sink_ = split(sink_, f.second);
    }

    return sources_[e];
  }
 
  typedef Media::Filter (*FilterFactory)(); 
  static FilterFactory factories[];

  Sink sink_;
  Source sources_[last_encoding];
};
  
Encoders::FilterFactory Encoders::factories[] = {
  Media::identity_filter,
  Media::pcma_encoder,
  Media::pcmu_encoder,
  Media::ilbc_encoder
};


struct Adaptor : RefcountedT<Adaptor> {
  void update() {
    Sink sinks20[last_encoding];
    Sink sinks30[last_encoding];
    Sink sink;
    
    for(Sinks::iterator i = sinks_.begin(), e = sinks_.end(); i != e; ++i) {
      if(is_compatible(i->second, own_)) {
        sink = split(sink, i->first);
      }
      else {
        Encoding e = preferred(i->second);

        switch(i->second.ptime_) {
        case packet20ms:
          sinks20[e] = split(i->first, sinks20[e]);
          break;
        case packet30ms:
          sinks30[e] = split(i->first, sinks30[e]);
          break;
        case packetAny:
          if(own_.ptime_ == packet30ms)
            sinks30[e] = split(i->first, sinks30[e]);
          else
            sinks20[e] = split(i->first, sinks20[e]);
          break;
        }
      }
    }

    for(size_t i = 0; i < sizeof(sinks20) / sizeof(*sinks20); ++i) { 
      if(!!sinks20[i])
        getEncoderSource(Encoding(i), packet20ms)(sinks20[i]);
    }

    for(size_t i = 0; i < sizeof(sinks30) / sizeof(*sinks30); ++i) {
      if(!!sinks30[i])
        getEncoderSource(Encoding(i), packet30ms)(sinks30[i]);
    }

    if(!!encoders_[0].first || !!encoders_[1].first) {
      if(!!jitter_.first)
        jitter_ = jitter();

      if(!!decoder_.first) {
        decoder_ = decoder(own_.encoding_);
        jitter_.first(decoder_.second);
      }

      decoder_.first(split(encoders_[0].first, encoders_[1].first));
    }

    sink_ = split(sink, jitter_.second);
  }

  void push(Packet const& packet) { sink_(packet); }
  
  typedef std::list<std::pair<Media::Sink, Media::Formats> > Sinks; 
  Sinks sinks_;

private:
  Source getEncoderSource(Encoding e, Packetization p) {
    std::pair<Sink, Encoders>* encoder = encoders_;
    if(p == packet30ms) 
      encoder = &encoders_[1];
    
    Source s = encoder->second.get(e);
    if(!encoder->first) {
      Filter f = packetizer(p);
      f.first(encoder->second.sink_);
      encoder->first = f.second; 
    }

    return s;
  }

  Sink sink_;

  Media::Format own_;
  
  Media::Filter jitter_;
  
  Media::Filter decoder_;

  std::pair<Sink, Encoders> encoders_[2];
};

boost::intrusive_ptr<Adaptor> make_adaptor() { return new Adaptor(); }

Sink get_sink(boost::intrusive_ptr<Adaptor> const& a) { return boost::bind(&Adaptor::push, a, _1); }

void set_sinks(boost::intrusive_ptr<Adaptor> const& a, std::list<std::pair<Sink, Formats> >& sinks) {
  a->sinks_.splice(a->sinks_.begin(), sinks);
  a->update();
}

void intrusive_ptr_add_ref(Adaptor* t) { intrusive_ptr_add_ref(static_cast<RefcountedT<Adaptor>*>(t)); }
void intrusive_ptr_release(Adaptor* t) { intrusive_ptr_release(static_cast<RefcountedT<Adaptor>*>(t)); }

Filter decoder(Encoding e) {
  typedef Filter (*Factory)();
  static Factory factories[] = {
    identity_filter,
    pcma_decoder,
    pcmu_decoder,
    ilbc_decoder
  };

  return factories[e]();
}


template<typename T>
struct Transformer : RefcountedT<Transformer<T> > {
  Transformer(T const& t) : t_(t) {}

  void push(Packet const& packet) {
    if(sink_) 
      sink_(t_(packet));
    else
      t_(packet);
  }

  void set_sink(Sink const& s) {
    sink_ = s;
  }

  T t_;
  Sink sink_;
};

template<typename T>
Filter make_transformer(T const& t) {
  boost::intrusive_ptr<Transformer<T> > tr(new Transformer<T>(t));
  return std::make_pair(boost::bind(&Transformer<T>::set_sink, tr, _1), boost::bind(&Transformer<T>::push, tr, _1));
}


template<typename T> T identity(T t) { return t; }

Filter identity_filter() {
  return make_transformer((Packet (*)(Packet))identity);
}

struct Packetizer : RefcountedT<Packetizer> {
  Packetizer(posix_time::time_duration const& goal) : goal_(goal) {}

  void push(Packet const& p) {
    packets_.push_back(p);
    Packets t;
    posix_time::time_duration d = posix_time::milliseconds(0);
    for(;!packets_.empty() && d < goal_ + offset_;) {
      t.splice(t.end(), packets_, packets_.begin()); 
      d += t.back().duration_;
    }

    if(d >= goal_ + offset_) {
      Packet r(t.front().srcid_, linear, t.front().ts_ + offset_, goal_, make_payload(sizeof(short) * goal_.total_microseconds()/125));

      short* d = (short*)r.payload_->data_;
      size_t offset = offset_.total_microseconds() / 125;
      size_t n = goal_.total_microseconds() / 125;

      for(; n > 0;) {
        size_t c = t.front().payload_->size_ / 2 - offset;

        if(t.front().payload_->size_ / 2 - offset >= n) {
          c = n;
          offset_ = posix_time::microseconds((offset + n) * 125);
        }

        memcpy(d, (short*)t.front().payload_->data_ + offset, c*2);
        n -= c;
        d += c;
        offset = 0;
        t.pop_front();
      }

      if(sink_) sink_(r);
    }
    else {
      packets_.splice(packets_.begin(), t);
    }
  }

  void set_sink(Sink const& s) {
    sink_ = s;
  }

  posix_time::time_duration goal_;

  typedef std::list<Packet> Packets;
  Packets packets_;
  posix_time::time_duration offset_;

  Sink sink_;
};

Filter packetizer(Packetization p) {
  if(p == packetAny)
    return identity_filter();

  boost::intrusive_ptr<Packetizer> z(new Packetizer(posix_time::milliseconds(p)));
  return std::make_pair(boost::bind(&Packetizer::set_sink, z, _1), boost::bind(&Packetizer::push, z, _1));
}

void splitter(Sink const& a, Sink const& b, Packet const& p) {
  a(p);
  b(p);
}

Sink split(Sink const& a, Sink const& b) {
  return boost::bind(splitter, a, b, _1);
}


struct Jitter : RefcountedT<Jitter> {
  Jitter(Clock const& clock) : clock_(clock) {}

  void tick(posix_time::ptime const& ptime) {
    if(delay_.is_special()) {
      if(!packets_.empty()) {
        delay_ = ptime - packets_.begin()->ts_ + posix_time::milliseconds(60);
        tick(ptime);
        return;
      }
    }
    else {
      if(!packets_.empty()) {
        posix_time::ptime ts = packets_.begin()->ts_ + delay_;

        if(ts == ptime) {
          Packet p(*packets_.begin());
          packets_.erase(packets_.begin());
          p.ts_ = ptime;
          emit(p);
        }
        else if(ts < ptime && ts - ptime < posix_time::milliseconds(150)) {
          delay_ = ptime - packets_.begin()->ts_ + lastDuration_*3;
          tick(ptime);
          return;
        }
        else
          emit(Packet(this, lastEncoding_, ptime, lastDuration_));
      } 
      else
        emit(Packet(this, lastEncoding_, ptime, lastDuration_));
    
      clock_(ptime + lastDuration_, boost::bind(&Jitter::tick, boost::intrusive_ptr<Jitter>(this), _1));
    }
  }

  void emit(Packet const& p) {
    if(!!sink_) sink_(p);
  }

  void push(Packet const& packet) {
    lastEncoding_ = packet.encoding_;
    lastDuration_ = packet.duration_;
    packets_.insert(packet);
  }

  void set_sink(Sink const& sink) {
    sink_ = sink;
  }

  Sink sink_;

  struct OrderPackets : std::binary_function<bool, Packet, Packet> {
    bool operator()(Packet const& a, Packet const& b) const { return a.ts_ < b.ts_; }
  };

  Clock clock_;
  typedef std::set<Packet, OrderPackets> Packets;
  Packets packets_;
  posix_time::time_duration delay_;
  Encoding lastEncoding_;
  posix_time::time_duration lastDuration_;
};

Filter jitter(Clock const& clock) {
  boost::intrusive_ptr<Jitter> j(new Jitter(clock));

  return std::make_pair(boost::bind(&Jitter::set_sink, j, _1), boost::bind(&Jitter::push, j, _1));
}

struct SystemClock : RefcountedT<SystemClock> {
  SystemClock() : timer_(g_io) {}

  static void handler(boost::function<void (posix_time::ptime const&)> const& f, posix_time::ptime const& ts, boost::system::error_code const& error) {
    if(!error)
      f(ts);
  }

  void set(posix_time::ptime const& ts, boost::function<void (posix_time::ptime const&)> const& f) {
    timer_.expires_at(ts);
    timer_.async_wait(boost::bind(handler, f, ts, _1));
  }

  asio::deadline_timer timer_;
}; 

Clock system_clock() {
  boost::intrusive_ptr<SystemClock> c(new SystemClock());
  
  return boost::bind(&SystemClock::set, c, _1, _2);
}


struct Rtp : RefcountedT<Rtp> {
  Rtp() : socket_(g_io), rssrc_(0) {}

  void enqueue_recv() {
    boost::intrusive_ptr<Payload> payload = make_payload(40*8);
    boost::array<asio::mutable_buffer,2> buffers;
    buffers[0] = asio::mutable_buffer(&rhdr_, sizeof(rhdr_));
    buffers[1] = asio::mutable_buffer(payload->data_, payload->size_);
    socket_.async_receive(buffers, boost::bind(&Rtp::on_packet_recv, boost::intrusive_ptr<Rtp>(this), payload, _1, _2));
  }

  void on_packet_recv(boost::intrusive_ptr<Payload> payload, boost::system::error_code const& ec, size_t bytes) {
    if(bytes == 0)
      return;

    if(bytes < sizeof(rhdr_) + rhdr_.cc*sizeof(boost::uint32_t))
      return;
    
    rhdr_.timestamp = ntohl(rhdr_.timestamp);
    rhdr_.seq = ntohs(rhdr_.seq);

    if(rhdr_.ssrc != rssrc_) { //resync
      rssrc_ = rhdr_.ssrc;

      rts_ = posix_time::microsec_clock::universal_time() - rtp_timestamp_to_duration(rhdr_.timestamp);  
    }

    if(!!sink_) {
      payload->size_ = bytes-sizeof(rhdr_);

      if(rhdr_.cc != 0) {
        payload->size_ -= rhdr_.cc*sizeof(boost::uint32_t);
        memmove(payload->data_, payload->data_+rhdr_.cc*sizeof(boost::uint32_t), payload->size_);
      }

      Encoding enc = rmap_(rhdr_.pt); 

      sink_(Packet(this, enc, rts_ + rtp_timestamp_to_duration(rhdr_.timestamp), size_to_duration(enc, payload->size_)));
      enqueue_recv();
    }
  }
 
  static posix_time::time_duration rtp_timestamp_to_duration(boost::uint32_t ts) {
    return posix_time::microseconds(125)*ts;
  }
 
  void write_handler(Packet, boost::system::error_code const&, size_t) {}
  
  void push(Packet const& packet) {
    shdr_.timestamp = htonl(duration_to_rtp_timestamp(packet.ts_ - sts_));

    shdr_.seq = htons(ntohs(shdr_.seq)+1);
    shdr_.pt = smap_(packet.encoding_); 

    boost::array<asio::const_buffer, 2> buffers;
    buffers[0] = asio::const_buffer(&shdr_, sizeof(shdr_));
    buffers[1] = asio::const_buffer(packet.payload_->data_, packet.payload_->size_);

    socket_.async_send(buffers, boost::bind(&Rtp::write_handler, boost::intrusive_ptr<Rtp>(this), packet, _1, _2));
  }

  static boost::uint32_t duration_to_rtp_timestamp(posix_time::time_duration const& d) {
    return d.total_microseconds()/125;
  }

  void set_sink(Sink sink) {
    std::swap(sink_, sink);

    if(!sink && !!sink_)
      enqueue_recv();
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

  asio::ip::udp::socket socket_;
  
  RtpHeader rhdr_;
  posix_time::ptime rts_;
  boost::uint32_t rssrc_;
  boost::function<Encoding (int)> rmap_;

  Sink sink_;

  RtpHeader shdr_;
  posix_time::ptime sts_;
  boost::function<int (Encoding)> smap_;
};

void intrusive_ptr_add_ref(Rtp* rtp) {
  intrusive_ptr_add_ref(static_cast<RefcountedT<Rtp>*>(rtp));
}

void intrusive_ptr_release(Rtp* rtp) {
  intrusive_ptr_release(static_cast<RefcountedT<Rtp>*>(rtp));
}

boost::intrusive_ptr<Rtp> make_rtp() {
  return new Rtp();
}

Sink get_sink(boost::intrusive_ptr<Rtp> const& rtp) {
  return boost::bind(&Rtp::push, rtp, _1);
}

Source get_source(boost::intrusive_ptr<Rtp> const& rtp) {
  return boost::bind(&Rtp::set_sink, rtp, _1);
}

void set_rtp_map(boost::intrusive_ptr<Rtp> const& rtp, boost::function<Encoding (int)> const& map) {
  rtp->rmap_ = map;
}

void set_rtp_map(boost::intrusive_ptr<Rtp> const& rtp, boost::function<int (Encoding)> const& map) {
  rtp->smap_ = map;
}

asio::ip::udp::endpoint get_local_endpoint(boost::intrusive_ptr<Rtp> const& rtp) {
  return rtp->socket_.local_endpoint();
}

void set_local_endpoint(boost::intrusive_ptr<Rtp> const& rtp, asio::ip::udp::endpoint const& ep) {
  rtp->socket_.bind(ep);
}

void set_remote_endpoint(boost::intrusive_ptr<Rtp> const& rtp, asio::ip::udp::endpoint const& ep) {
  rtp->socket_.connect(ep);
}

asio::io_service g_file_io;

struct FileSource : RefcountedT<FileSource> {
  FileSource(const char* file, Format f, boost::function<void (void)> const& eof, Clock const& clock) : 
    clock_(clock),
    format_(f),
    eof_(eof),
    fd_(g_file_io)
  {
    int fd = open(file, O_RDONLY);

    if(-1 == fd) {
      throw boost::system::system_error(boost::system::errc::make_error_code(boost::system::errc::errc_t(errno)));
    }
  
    fd_.assign(fd);
  }

  void start() {
    clock_(posix_time::microsec_clock::universal_time(), boost::bind(&FileSource::tick, boost::intrusive_ptr<FileSource>(this), _1));
  }

  void tick(posix_time::ptime const& ts) {
    boost::intrusive_ptr<Payload> payload = make_payload(payload_size());

    async_read(fd_, asio::mutable_buffers_1(payload->data_, payload->size_),
      boost::bind(&FileSource::read_complete, boost::intrusive_ptr<FileSource>(this), 
        Packet(this, format_.encoding_, ts, duration(format_.ptime_), payload), _1, _2));
  }

  void read_complete(Packet const& packet, boost::system::error_code const& ec, size_t bytes) {
    if(ec) {
      g_io.post(eof_);
    }
    else {
      g_io.post(boost::bind(&FileSource::push_packet, boost::intrusive_ptr<FileSource>(this), packet));
      g_io.post(boost::bind(clock_, packet.ts_ + packet.duration_, 
        boost::function<void (posix_time::ptime const&)>(boost::bind(&FileSource::tick, boost::intrusive_ptr<FileSource>(this), _1))));
    }
  }

  size_t payload_size() {
    switch(format_.encoding_) {
    case linear:
      return format_.ptime_ == packet30ms ? 480 : 320;
    case pcma:
    case pcmu:
      return format_.ptime_ == packet30ms ? 240: 160;
    case ilbc:
      return format_.ptime_ == packet30ms ? 50 : 38;
    default:
      assert(0);
      return 160;
    }
  }

  void push_packet(Packet const& packet) {
    if(sink_) sink_(packet);
  }

  void set_sink(Sink const& sink) {
    sink_ = sink;
  }

  Clock clock_;
  Format format_;
  boost::function<void () > eof_;
  asio::posix::stream_descriptor fd_;

  Sink sink_;
};

boost::intrusive_ptr<FileSource> make_file_source(const char* filename, Format const& fmt, boost::function<void ()> const& eof, Clock const& clock) {
  boost::intrusive_ptr<FileSource> fs = new FileSource(filename, fmt, eof, clock);
  fs->start();
  return fs;
}

Source get_source(boost::intrusive_ptr<FileSource> const& fs) {
  return boost::bind(&FileSource::set_sink, fs, _1);
}

void intrusive_ptr_add_ref(FileSource* fs) {
  return intrusive_ptr_add_ref(static_cast<RefcountedT<FileSource>*>(fs));
}

void intrusive_ptr_release(FileSource* fs) {
  return intrusive_ptr_release(static_cast<RefcountedT<FileSource>*>(fs));
}

Packet decode_g711(Packet const& packet, short (*decoder)(unsigned char)) {
  Packet dcd(packet.srcid_, linear, packet.ts_, packet.duration_, make_payload(packet.duration_.total_microseconds() / 125 *2));
  if(packet.payload_)
    std::transform(packet.payload_->data_, packet.payload_->data_ + packet.payload_->size_, (short*)dcd.payload_->data_, decoder);
  else
    memset(dcd.payload_->data_, 0, dcd.payload_->size_);
  
  return dcd;
} 

Filter pcma_decoder() {
  return make_transformer(boost::bind(decode_g711, _1, alaw2linear));
}

Filter pcmu_decoder() {
  return make_transformer(boost::bind(decode_g711, _1, ulaw2linear));
}

Packet encode_g711(Packet const& packet, Encoding e, unsigned char (*encoder)(short)) {
  Packet r(packet.srcid_, e, packet.ts_, packet.duration_);

  if(packet.payload_) {
    r.payload_ = make_payload(packet.duration_.total_microseconds() / 125);

    std::transform((short*)packet.payload_->data_, (short*)(packet.payload_->data_ + packet.payload_->size_), r.payload_->data_, encoder);
  }

  return r;
}

Filter pcma_encoder() {
  return make_transformer(boost::bind(encode_g711, _1, pcma, linear2alaw));
}

Filter pcmu_encoder() {
  return make_transformer(boost::bind(encode_g711, _1, pcmu, linear2ulaw));
}


struct IlbcDecoder {
  IlbcDecoder() {
    initDecode(&state_, 20, 1);
  }

  Packet operator()(Packet const& p) {
    if(p.duration_.total_milliseconds() != state_.mode)
      initDecode(&state_, p.duration_.total_milliseconds(), 1);

    float dec[240];
    if(p.payload_)
      iLBC_decode(dec, (unsigned char*)p.payload_->data_, &state_, 1);
    else
      iLBC_decode(dec, 0, &state_, 0);

    Packet r(p.srcid_, linear, p.ts_, p.duration_, make_payload(state_.blockl));
    std::copy(dec, dec + state_.blockl, r.payload_->data_);
    return r;
  }

  iLBC_Dec_Inst_t state_;
};

Filter ilbc_decoder() {
  return make_transformer(IlbcDecoder());
}


struct IlbcEncoder {
  IlbcEncoder() {
    initEncode(&state_, 20);
  }

  Packet operator()(Packet const& p) {
    if(p.duration_.total_milliseconds() != state_.mode)
      initEncode(&state_, p.duration_.total_milliseconds());
    
    float data[240];
    std::copy(p.payload_->data_, p.payload_->data_ + state_.mode, data);

    Packet r(p.srcid_, ilbc, p.ts_, p.duration_, make_payload(state_.mode == 30 ? 50 : 38));
    iLBC_encode((unsigned char*)r.payload_->data_, data, &state_);
    
    return r;
  }

  iLBC_Enc_Inst_t state_;
};

Filter ilbc_encoder() {
  return make_transformer(IlbcEncoder());
}


std::auto_ptr<asio::io_service::work> g_work;
boost::thread g_thread;

#ifdef __MACH__
void set_realtime_priority() {
  mach_timebase_info_data_t tb;

  mach_timebase_info(&tb);

  thread_time_constraint_policy policy;

  policy.period = 10*1000*1000 * tb.denom / tb.numer;
  policy.computation = 2*1000*1000 * tb.denom / tb.numer;
  policy.constraint= 5 * 1000*1000 * tb.denom / tb.numer;
  policy.preemptible = 0;
 
  kern_return_t ret = thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&policy, THREAD_TIME_CONSTRAINT_POLICY_COUNT);

  assert(KERN_SUCCESS == ret);
}
#endif

#ifdef _WIN32_WINNT
void set_realtime_priority() {
  ::SetThreadPriority(::GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
}
#endif

void thread_proc() {
  set_realtime_priority();
  g_io.run();
}

void start() {
  g_work = std::auto_ptr<asio::io_service::work>(new asio::io_service::work(g_io));
  boost::thread thread(thread_proc);
  g_thread.swap(thread);
}

void stop() {
  g_io.stop();
  g_thread.join();
}

}

