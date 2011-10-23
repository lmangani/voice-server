#include "media.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

#include <iostream>

#ifdef __MACH__
#include <mach/mach_init.h>
#include <mach/thread_policy.h>
#include <mach/thread_act.h>
#include <mach/mach_time.h>
#endif

unsigned char linear2ulaw(short);
unsigned char linear2alaw(short);

std::ostream& operator << (std::ostream & os, Media::PacketPtr const& p) {
  return os << "{" << p->payload_type()->name() << ", " << p->timestamp() << "," << p->duration() << "}";
}

namespace Media {
asio::io_service g_io;

struct _8000HzBasedTraits : PayloadTraits {
  posix_time::time_duration default_packet_duration() const {
    return posix_time::milliseconds(20);
  } 

  posix_time::time_duration rtp_timestamp_to_duration(boost::uint32_t ts) const {
    return posix_time::microseconds(125) * static_cast<int>(ts);
  }

  boost::uint32_t duration_to_rtp_timestamp(posix_time::time_duration const& duration) const {
    return duration.total_microseconds() / 125; 
  }
};

PayloadTraits const* get_g711_traits(bool u) {
  using namespace posix_time;

  struct G711Base : _8000HzBasedTraits {
    size_t frame_size() const { return 1; }
    
    time_duration frame_duration() const {
      return microseconds(125);
    }
  };

  static struct G711A : G711Base {
    const char* name() const {
      return "PCMA/8000";
    }

    void fill_silence(void* data, size_t size) const {
      memset(data, 0xd5, size);
    }
  } g711a;

  static struct G711B : G711Base {
    const char* name() const {
      return "PCMU/8000";
    }

    void fill_silence(void* data, size_t size) const {
      memset(data, 0xff, size);
    }
  } g711u;

  return u ? static_cast<PayloadType>(&g711u) : &g711a;
}

PayloadTraits const* get_g711a_traits() {
  return get_g711_traits(false);
}

PayloadTraits const* get_g711u_traits() {
  return get_g711_traits(true);
}

PayloadTraits const* get_l16_traits() {
  using namespace posix_time;

  static struct L16 : _8000HzBasedTraits {
    const char* name() const {
      return "L16/8000";
    }

    size_t frame_size() const { return 2; }
    
    time_duration frame_duration() const {
      return microseconds(125);
    }

    void fill_silence(void* data, size_t size) const {
      memset(data, 0, size);
    }
  } l16;

  return &l16;
}

struct TelephoneEvent {
  unsigned event:8;
  unsigned volume:6;
  unsigned r:1;
  unsigned e:1;
  unsigned duration:16;
};

PayloadTraits const* get_rfc2833_traits() {
  using namespace posix_time;

  static struct TE : _8000HzBasedTraits {
    const char* name() const {
      return "telephone-event/8000";
    }
    
    time_duration frame_duration() const {
      return microseconds(0);
    }

    size_t frame_size() const { return sizeof(TelephoneEvent); }

    void fill_silence(void*, size_t) const {
      assert(0);
    }
  } te;

  return &te;
}

PacketPtr G711aDecoder::operator()(PacketPtr const& packet) const {
  if(packet->payload_type() != G711A)
    return packet;


  PacketPtr decoded = make_packet(L16, packet->timestamp(), packet->duration());

  static boost::int16_t table[256] = { 
     -5504, -5248, -6016, -5760, -4480, -4224, -4992, -4736, 
     -7552, -7296, -8064, -7808, -6528, -6272, -7040, -6784, 
     -2752, -2624, -3008, -2880, -2240, -2112, -2496, -2368, 
     -3776, -3648, -4032, -3904, -3264, -3136, -3520, -3392, 
     -22016,-20992,-24064,-23040,-17920,-16896,-19968,-18944, 
     -30208,-29184,-32256,-31232,-26112,-25088,-28160,-27136, 
     -11008,-10496,-12032,-11520,-8960, -8448, -9984, -9472, 
     -15104,-14592,-16128,-15616,-13056,-12544,-14080,-13568, 
     -344,  -328,  -376,  -360,  -280,  -264,  -312,  -296, 
     -472,  -456,  -504,  -488,  -408,  -392,  -440,  -424, 
     -88,   -72,   -120,  -104,  -24,   -8,    -56,   -40, 
     -216,  -200,  -248,  -232,  -152,  -136,  -184,  -168, 
     -1376, -1312, -1504, -1440, -1120, -1056, -1248, -1184, 
     -1888, -1824, -2016, -1952, -1632, -1568, -1760, -1696, 
     -688,  -656,  -752,  -720,  -560,  -528,  -624,  -592, 
     -944,  -912,  -1008, -976,  -816,  -784,  -880,  -848, 
      5504,  5248,  6016,  5760,  4480,  4224,  4992,  4736, 
      7552,  7296,  8064,  7808,  6528,  6272,  7040,  6784, 
      2752,  2624,  3008,  2880,  2240,  2112,  2496,  2368, 
      3776,  3648,  4032,  3904,  3264,  3136,  3520,  3392, 
      22016, 20992, 24064, 23040, 17920, 16896, 19968, 18944, 
      30208, 29184, 32256, 31232, 26112, 25088, 28160, 27136, 
      11008, 10496, 12032, 11520, 8960,  8448,  9984,  9472, 
      15104, 14592, 16128, 15616, 13056, 12544, 14080, 13568, 
      344,   328,   376,   360,   280,   264,   312,   296, 
      472,   456,   504,   488,   408,   392,   440,   424, 
      88,    72,   120,   104,    24,     8,    56,    40, 
      216,   200,   248,   232,   152,   136,   184,   168, 
      1376,  1312,  1504,  1440,  1120,  1056,  1248,  1184, 
      1888,  1824,  2016,  1952,  1632,  1568,  1760,  1696, 
      688,   656,   752,   720,   560,   528,   624,   592, 
      944,   912,  1008,   976,   816,   784,   880,   848 
  };

  for(size_t i = 0; i < packet->size(); ++i) {
    reinterpret_cast<boost::int16_t*>(decoded->samples())[i] = table[packet->samples()[i]];
  }

  return decoded;
}

PacketPtr G711uDecoder::operator()(PacketPtr const& packet) const {
  if(packet->payload_type() != G711U)
    return packet;

  PacketPtr decoded = make_packet(L16, packet->timestamp(), packet->duration());

  static boost::int16_t table[256] = { 
    -32124,-31100,-30076,-29052,-28028,-27004,-25980,-24956, 
    -23932,-22908,-21884,-20860,-19836,-18812,-17788,-16764, 
    -15996,-15484,-14972,-14460,-13948,-13436,-12924,-12412, 
    -11900,-11388,-10876,-10364, -9852, -9340, -8828, -8316, 
     -7932, -7676, -7420, -7164, -6908, -6652, -6396, -6140, 
     -5884, -5628, -5372, -5116, -4860, -4604, -4348, -4092, 
     -3900, -3772, -3644, -3516, -3388, -3260, -3132, -3004, 
     -2876, -2748, -2620, -2492, -2364, -2236, -2108, -1980, 
     -1884, -1820, -1756, -1692, -1628, -1564, -1500, -1436, 
     -1372, -1308, -1244, -1180, -1116, -1052,  -988,  -924, 
      -876,  -844,  -812,  -780,  -748,  -716,  -684,  -652, 
      -620,  -588,  -556,  -524,  -492,  -460,  -428,  -396, 
      -372,  -356,  -340,  -324,  -308,  -292,  -276,  -260, 
      -244,  -228,  -212,  -196,  -180,  -164,  -148,  -132, 
      -120,  -112,  -104,   -96,   -88,   -80,   -72,   -64, 
       -56,   -48,   -40,   -32,   -24,   -16,    -8,     0, 
     32124, 31100, 30076, 29052, 28028, 27004, 25980, 24956, 
     23932, 22908, 21884, 20860, 19836, 18812, 17788, 16764, 
     15996, 15484, 14972, 14460, 13948, 13436, 12924, 12412, 
     11900, 11388, 10876, 10364,  9852,  9340,  8828,  8316, 
      7932,  7676,  7420,  7164,  6908,  6652,  6396,  6140, 
      5884,  5628,  5372,  5116,  4860,  4604,  4348,  4092, 
      3900,  3772,  3644,  3516,  3388,  3260,  3132,  3004, 
      2876,  2748,  2620,  2492,  2364,  2236,  2108,  1980, 
      1884,  1820,  1756,  1692,  1628,  1564,  1500,  1436, 
      1372,  1308,  1244,  1180,  1116,  1052,   988,   924, 
       876,   844,   812,   780,   748,   716,   684,   652, 
       620,   588,   556,   524,   492,   460,   428,   396, 
       372,   356,   340,   324,   308,   292,   276,   260, 
       244,   228,   212,   196,   180,   164,   148,   132, 
       120,   112,   104,    96,    88,    80,    72,    64, 
        56,    48,    40,    32,    24,    16,     8,     0  
  };

  for(size_t i = 0; i < packet->size(); ++i) {
    *reinterpret_cast<boost::int16_t*>(decoded->samples()) = table[packet->samples()[i]];
  }

  return decoded;
}

PacketPtr G711aEncoder::operator()(PacketPtr const& packet) const {

  if(packet->payload_type() != L16)
    return packet;

  PacketPtr encoded = make_packet(G711A, packet->timestamp(), packet->duration());

  for(size_t i = 0; i < encoded->size(); ++i) {
    encoded->samples()[i] = linear2alaw(reinterpret_cast<boost::int16_t*>(packet->samples())[i]);
  }
  return encoded;
}

PacketPtr G711uEncoder::operator()(PacketPtr const& packet) const {
  if(packet->payload_type() != L16)
    return packet;

  PacketPtr encoded = make_packet(G711U, packet->timestamp(), packet->duration());

  for(size_t i = 0; i < encoded->size(); ++i) {
    encoded->samples()[i] = linear2ulaw(reinterpret_cast<boost::int16_t*>(packet->samples())[i]);
  }
  return encoded;
}

Source mix2(Source a, Source b) {
  struct Sk : Detail::SinkVTable {
    void operator()(PacketPtr const&  p) {
      TimerGuard tg;
      if(p_) {
        for(size_t i = 0; i < std::min(p->size(), p_->size()) / sizeof(boost::uint16_t); ++i) {
          *(reinterpret_cast<boost::uint16_t*>(p_->samples()) + i) += *(reinterpret_cast<boost::uint16_t*>(p->samples())+i);
        }

        sink_(p_);
        p_ = 0; 
      }
      else {
        p_ = p;
      }
    }

    Sink sink_;
    PacketPtr p_;
  };

  struct Src : Detail::SourceVTable {
    Src(Source a, Source b) : a_(a), b_(b), sink_(new Sk()) {}

    void operator()(Sink const& s) {
      remember_sink(s);
      a_(sink_);
      b_(sink_);
    }

    void remember_sink(Sink s) {
      static_cast<Sk*>(sink_.v_.get())->sink_ = s;
    }
  
    Source a_;
    Source b_;
    Sink sink_;
  };

  return Source(new Src(a, b));
}
Sink split2(Sink a, Sink b) {
  struct Sk : Detail::SinkVTable {
    Sk(Sink a, Sink b) : a_(a), b_(b) {}

    void operator()(PacketPtr const& p) {
      a_(p);
      b_(p);
    }

    Sink a_;
    Sink b_;
  };

  return Sink(new Sk(a, b));
}

struct JitterBuffer : Refcounted<JitterBuffer> {
  JitterBuffer(PayloadType pt, posix_time::time_duration const& latency) : latency_(latency), pt_(pt) {}

  typedef boost::circular_buffer<PacketPtr> buffer_list_t;
    
  void push(PacketPtr const& p) {
    ensure_buffers_initialized();

    if(p->payload_type() != pt_)
      return;

    if(p->timestamp() < ts_)
      return;

    if(p->timestamp() + p->duration() > buffers_.back()->timestamp() + buffers_.back()->duration())
      return;

    buffer_list_t::iterator i = buffers_.begin();
    for(;p->timestamp() >= (*i)->timestamp()+(*i)->duration();++i);

    size_t offset = duration_to_size(pt_, p->timestamp() - (*i)->timestamp());
    size_t copy = std::min((*i)->size() - offset, p->size());

    memcpy((*i)->samples() + offset, p->samples(), copy);
      
    for(size_t copied = copy; copied != p->size(); copied += copy) {
      ++i;
      copy = std::min((*i)->size(), p->size() - copied);
      memcpy((*i)->samples(), p->samples() + copied, copy); 
    }
  }

  void pull(Sink sink) {
    ensure_buffers_initialized();

    PacketPtr p = buffers_.front();
    buffers_.pop_front();
    ts_ += p->duration();
    buffers_.push_back(make_silence_packet(pt_, buffers_.back()->timestamp() + buffers_.back()->duration()));
    sink(p);
  }

  void ensure_buffers_initialized() {
    if(ts_.is_special()) {
      int packets = ((latency_ * 2).total_milliseconds() + pt_->default_packet_duration().total_milliseconds()-1) / pt_->default_packet_duration().total_milliseconds();
      buffers_ = boost::circular_buffer<PacketPtr>(packets);
      ts_ = posix_time::microsec_clock::universal_time() - latency_;
      for(int i =0; i < packets; ++i) {
        buffers_.push_back(make_silence_packet(pt_, ts_ + pt_->default_packet_duration() * i));
      }
    }
  }

  boost::circular_buffer<PacketPtr> buffers_;
  posix_time::ptime ts_;
  posix_time::time_duration latency_;
  PayloadType pt_;
};

std::pair<Source, Sink> make_jitter_buffer(PayloadType pt, posix_time::time_duration const& latency) {
   boost::intrusive_ptr<JitterBuffer> p = new JitterBuffer(pt, latency);

  return std::make_pair(
    make_source(boost::bind(&JitterBuffer::pull, p, _1)),
    make_sink(boost::bind(&JitterBuffer::push, p, _1)));
}


int default_payload_to_rtp(PayloadType pt) {
  if(pt == G711A)
    return 8;
  else if(pt == G711U) 
    return 0;
  else if(pt == RFC2833) 
    return 101;

  return -1;
}

PayloadType default_rtp_to_payload(int pt) {
  switch(pt) {
  case 0:
    return G711U;
  case 8:
    return G711A;
  default:
    return 0;
  }
}

Rtp::Rtp() : socket_(g_io), smap_(default_payload_to_rtp), rmap_(default_rtp_to_payload) {
  memset(&shdr_, 0, sizeof(shdr_));

  socket_.open(asio::ip::udp::v4());

  shdr_.seq = rand();
  shdr_.version = 2;
  shdr_.ssrc = rand();
  
  sts_ = posix_time::microsec_clock::universal_time() - posix_time::microseconds(rand());
}

void rtp_write_handler(boost::intrusive_ptr<Rtp> rtp, PacketPtr, boost::system::error_code const&, size_t) {}

void Rtp::push(PacketPtr const& packet) {
  //std::cerr << packet << ":" << posix_time::microsec_clock::universal_time() - packet->timestamp() << std::endl;
  TimerGuard tg;

  shdr_.timestamp = htonl(duration_to_rtp_timestamp(packet->payload_type(), packet->timestamp() - sts_));

  shdr_.seq = htons(ntohs(shdr_.seq)+1);
  shdr_.pt = smap_(packet->payload_type()); 

  boost::array<asio::const_buffer, 2> buffers;
  buffers[0] = asio::const_buffer(&shdr_, sizeof(shdr_));
  buffers[1] = asio::const_buffer(packet->samples(), packet->size());

  socket_.async_send(buffers, boost::bind(rtp_write_handler, boost::intrusive_ptr<Rtp>(this), packet, _1, _2));
}

void Rtp::connect(asio::ip::udp::endpoint const& ep) {
  socket_.connect(ep);
}

void Rtp::bind(asio::ip::udp::endpoint const& ep) {
  socket_.bind(ep);
}

void Rtp::set_sink(Sink const& sink) {
  if(sink_.empty()) {
    enqueue_recv();
  }

  sink_ = sink;
}

void Rtp::enqueue_recv() {
  PacketPtr packet = make_packet(duration_to_size(G711A, posix_time::milliseconds(40)));
  boost::array<asio::mutable_buffer,2> buffers;
  buffers[0] = asio::mutable_buffer(&rhdr_, sizeof(rhdr_));
  buffers[1] = asio::mutable_buffer(packet->samples(), packet->size());
  socket_.async_receive(buffers, boost::bind(&Rtp::on_packet_recv, boost::intrusive_ptr<Rtp>(this), packet, _1, _2));
}

void Rtp::on_packet_recv(PacketPtr const& packet, boost::system::error_code const& ec, size_t bytes) {
  if(bytes == 0)
    return;

  if(bytes < sizeof(rhdr_) + rhdr_.cc*sizeof(boost::uint32_t))
    return;

  PayloadType pt = rmap_(rhdr_.pt);

  if(pt == 0)
    return;

  packet->size_ = bytes-sizeof(rhdr_);

  if(rhdr_.cc != 0) {
    packet->size_ -= rhdr_.cc*sizeof(boost::uint32_t);
    memmove(packet->samples(), packet->samples()+rhdr_.cc*sizeof(boost::uint32_t), packet->size_);
  }

  rhdr_.timestamp = ntohl(rhdr_.timestamp);
  rhdr_.seq = ntohs(rhdr_.seq);

  if(rhdr_.ssrc != rssrc_) { //resync
    rssrc_ = rhdr_.ssrc;

    rts_ = posix_time::microsec_clock::universal_time() - rtp_timestamp_to_duration(pt, rhdr_.timestamp);  
  }

  packet->timestamp_ = rts_ + rtp_timestamp_to_duration(pt, rhdr_.timestamp);

  packet->duration_ = size_to_duration(pt, packet->size_);

  packet->payload_type_ = pt;

  if(!!sink_) {
    sink_(packet.get());
    enqueue_recv();
  }
}

void Rtp::set_source(Source const& source) {
  if(!!puller_) {
    puller_->stop();
    puller_ = 0;
  }

  if(!!source) {
    puller_ = new Detail::Puller(source, make_sink(boost::intrusive_ptr<Rtp>(this)));
    puller_->start();
  }
}

void Rtp::close() {
  return socket_.close();
}

asio::ip::udp::endpoint Rtp::local_endpoint() {
  return socket_.local_endpoint();
}

#ifndef _WIN32_WINNT
extern asio::io_service g_file_io;
#endif


#ifdef _WIN32_WINNT
FileSource::FileSource(const char* file, PayloadType pt, boost::function<void ()> const& f) : file_(g_io), pt_(pt), on_eof_(f) {
  HANDLE h = ::CreateFileA(
    file,
    GENERIC_READ,
    FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
    0,
    OPEN_EXISTING,
    FILE_FLAG_OVERLAPPED,
    0   
  );

  if(INVALID_HANDLE_VALUE == h)
    throw boost::system::system_error(boost::system::error_code(::GetLastError(), boost::system::system_category));

  file_.assign(h);
}
#else
FileSource::FileSource(const char* file, PayloadType pt, boost::function<void ()> const& f) : file_(g_file_io), pt_(pt), on_eof_(f) {
  int fd = open(file, O_RDWR);

  if(-1 == fd) {
    throw boost::system::system_error(boost::system::errc::make_error_code(boost::system::errc::errc_t(errno)));
  }
  
  file_.assign(fd);
}
#endif

#ifndef _WIN32_WINNT
void packet_read(boost::intrusive_ptr<FileSource> f, Sink const& sink, PacketPtr const& packet) {
  asio::async_read(f->file_, asio::mutable_buffers_1(packet->samples(), packet->size()),
    boost::bind(&FileSource::on_read_complete, f, sink, packet, _1, _2));
}
#endif

void FileSource::pull(Sink const& sink) {
  TimerGuard tg;

  if(ts_.is_special()) {
    ts_ = posix_time::microsec_clock::universal_time();
#ifdef _WIN32_WINNT
    begin_ts_ = ts_;
#endif
  }

  boost::intrusive_ptr<Packet> packet = make_packet(pt_, ts_);
  ts_ += packet->duration();

#ifdef _WIN32_WINNT
  asio::async_read_at(
    file_,
    duration_to_size(pt_, ts_ - begin_ts_),    
    asio::mutable_buffers_1(packet->samples(), packet->size()),
    boost::bind(&FileSource::on_read_complete, boost::intrusive_ptr<FileSource>(this), sink, packet, _1, _2));
#else
  g_file_io.post(boost::bind(packet_read, boost::intrusive_ptr<FileSource>(this), sink, packet));
#endif
}

void FileSource::on_read_complete(Sink const& sink, PacketPtr const& packet, boost::system::error_code const& ec, size_t bytes) {
  if(0 == bytes || ec) {
    if(!on_eof_.empty())
      on_eof_();
  }
  else {
    packet->size_ = bytes;
    packet->duration_ = size_to_duration(pt_, bytes);

    sink(packet);
  }
}

void FileSource::set_sink(Sink const& sink) {
  if(!!puller_) {
    puller_->stop();
    puller_ = 0;
  }

  if(!!sink) {
    puller_ = new Detail::Puller(make_source(boost::intrusive_ptr<FileSource>(this)), sink);
    puller_->start();
  }
}

void FileSource::close() {
  file_.close();
}


#ifdef _WIN32_WINNT
FileSink::FileSink(const char* file) : file_(g_io) {
  HANDLE h = ::CreateFileA(
    file,
    GENERIC_WRITE,
    FILE_SHARE_DELETE,
    0,
    CREATE_ALWAYS,
    FILE_FLAG_OVERLAPPED,
    0);

  if(INVALID_HANDLE_VALUE == h)
    throw boost::system::system_error(boost::system::error_code(::GetLastError(), boost::system::system_category));

  file_.assign(h);
}
#else
FileSink::FileSink(const char* file) : file_(g_file_io) {
  int fd = open(file, O_WRONLY | O_CREAT | O_TRUNC, 0666);

  if(-1 == fd) {
    throw boost::system::system_error(boost::system::errc::make_error_code(boost::system::errc::errc_t(errno)));
  }

  file_.assign(fd);
}
#endif

void packet_write_handler(PacketPtr const& , boost::system::error_code const&, size_t) {}

#ifndef _WIN32_WINNT
void packet_write(boost::intrusive_ptr<FileSink> f, PacketPtr const& packet) {
  asio::async_write(f->file_, asio::const_buffers_1(packet->samples(), packet->size()), boost::bind(packet_write_handler, packet, _1, _2));
}
#endif

void FileSink::push(PacketPtr const& packet) {
  TimerGuard tg;
#ifdef _WIN32_WINNT
  if(begin_ts_.is_special())
    begin_ts_ = packet->timestamp();
  asio::async_write_at(
    file_, 
    duration_to_size(packet->payload_type(), packet->timestamp()-begin_ts_),
    asio::const_buffers_1(packet->samples(), 
    packet->size()),
    boost::bind(packet_write_handler, packet, _1, _2));
#else
  g_file_io.post(boost::bind(packet_write, boost::intrusive_ptr<FileSink>(this), packet));
#endif
}

void FileSink::set_source(Source const& source) {
  if(puller_) {
    puller_->stop();
    puller_ = 0;
  }

  if(!!source) {
    puller_ = new Detail::Puller(source, make_sink(boost::intrusive_ptr<FileSink>(this)));
    puller_->start();
  }
}

void FileSink::close() {
  file_.close();
}

#ifndef _WINNT_WIN32
FileSocket::FileSocket(PayloadType pt) : socket_(g_io), pt_(pt) { }

void FileSocket::connect(const char* path) {
  socket_.connect(asio::local::stream_protocol::endpoint(path));
}

void FileSocket::set_source(Source const& source) {
  if(src_puller_) { 
    src_puller_->stop();
    src_puller_ = 0;
  }

  if(!!source) {
    src_puller_ = new Detail::Puller(source, make_sink(boost::intrusive_ptr<FileSocket>(this)));
    src_puller_->start();
  }
}

void FileSocket::push(PacketPtr const& packet) {
  asio::async_write(socket_, asio::const_buffers_1(packet->samples(), packet->size()), boost::bind(packet_write_handler, packet, _1, _2));
}

void FileSocket::on_read_complete(Sink const& sink, PacketPtr const& packet, boost::system::error_code const& ec, size_t bytes) {
  if(0 == bytes || ec) {
  }
  else {
    packet->size_ = bytes;
    packet->duration_ = size_to_duration(pt_, bytes);

    sink(packet);
  }
}

void FileSocket::set_sink(Sink const& sink) {
  if(!!sink_puller_) {
    sink_puller_->stop();
    sink_puller_ = 0;
  }

  if(!!sink) {
    sink_puller_ = new Detail::Puller(make_source(boost::intrusive_ptr<FileSocket>(this)), sink);
    sink_puller_->start();
  }
}

void FileSocket::pull(Sink const& sink) {
  TimerGuard tg;

  if(ts_.is_special()) {
    ts_ = posix_time::microsec_clock::universal_time();
  }

  boost::intrusive_ptr<Packet> packet = make_packet(pt_, ts_);
  ts_ += packet->duration();

  asio::async_read(socket_, asio::mutable_buffers_1(packet->samples(), packet->size()),
    boost::bind(&FileSocket::on_read_complete, boost::intrusive_ptr<FileSocket>(this), sink, packet, _1, _2));
}

void FileSocket::close() {
  set_sink(Sink());
  set_source(Source());

  socket_.close();
}

#endif

void TelephoneEventDetector::push(PacketPtr const& packet) {
  if(packet->payload_type() != RFC2833)
    return;

  TelephoneEvent* te = reinterpret_cast<TelephoneEvent*>(packet->samples());

  // filtering non-dtmf events
  if(te->event > 15)
    return;

  if(in_event_) {
    if(te->event != event_ || packet->timestamp() != ts_) {
      if(!!on_end_) on_end_(event_, ts_);

      event_ = te->event;
      ts_ = packet->timestamp();
        
      if(!!on_begin_) on_begin_(event_, ts_);
    }
    else if(te->e) {
      if(!!on_end_) on_end_(event_, ts_);
      in_event_ = false;
    }
  }
  else { 
    in_event_ = true;
    event_ = te->event;
    ts_ = packet->timestamp();
        
    if(!!on_begin_) on_begin_(event_, ts_);
  }

  timer_.cancel();
  timer_.expires_from_now(posix_time::milliseconds(100));
  timer_.async_wait(boost::bind(&TelephoneEventDetector::on_timeout, boost::intrusive_ptr<TelephoneEventDetector>(this), _1));
}

void TelephoneEventDetector::on_timeout(boost::system::error_code const& ec) {
  if(!ec && in_event_) {
    in_event_ = false;
    if(!!on_end_) on_end_(event_, ts_);
  }
}

boost::thread g_thread;

std::auto_ptr<asio::io_service::work> g_work;

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
#else
void set_realtime_priority() {
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

//Under Mac OS X async_write sometime takes up to 200 ms. This is unacceptable in main media thread.
//I'm pretty sure there is no such problem under Windows. Not sure about Linux though
asio::io_service g_file_io;
std::auto_ptr<asio::io_service::work> g_file_work;

boost::thread g_file_thread;

void start() {
  g_work = std::auto_ptr<asio::io_service::work>(new asio::io_service::work(g_io));

#ifndef _WIN32_WINNT
  g_file_work= std::auto_ptr<asio::io_service::work>(new asio::io_service::work(g_file_io));
  boost::thread file(boost::bind(&asio::io_service::run, boost::ref(g_file_io)));
  g_file_thread.swap(file);
#endif

  boost::thread thread(thread_proc);
  g_thread.swap(thread);
}

void stop() {
  g_io.stop();
  g_thread.join();
}

}

#ifdef MEDIA_TEST
int main(int c, char* argv[]) {
  return 0;
}
#endif

