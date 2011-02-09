#include <list>
#include <boost/intrusive_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/function.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/asio.hpp>

namespace Media {

namespace asio = boost::asio;
namespace posix_time = boost::posix_time;

typedef enum {
  first_encoding = 0,
  linear = first_encoding,
  pcma,
  pcmu,
  ilbc,
  last_encoding = ilbc
} Encoding;

typedef enum {
  packetAny = 0,
  packet20ms = 20,
  packet30ms = 30
} Packetization;

inline
posix_time::time_duration duration(Packetization p) {
  return packet30ms ? posix_time::milliseconds(30) : posix_time::milliseconds(20);
}

struct Format {
  Encoding encoding_;
  Packetization ptime_;
  bool dejittered_;
};

struct Formats {
  Formats() {}
  Formats(Encoding e, Packetization ptime, bool dejittered) : 
    mask_(1 << e), ptime_(ptime), dejittered_(dejittered)
  {}

  boost::uint32_t mask_;
  Packetization ptime_;
  bool dejittered_;
};

inline bool repacketization_needed(Formats const& formats, Format const& f) { return formats.ptime_ != packetAny && formats.ptime_ != f.ptime_; }

inline
bool is_compatible(Formats const& formats, Format const& f) {
  if(0 == (f.encoding_ & formats.mask_))
    return false;
  if(formats.dejittered_ && !f.dejittered_)
    return false;
  if(formats.ptime_ != packetAny && formats.ptime_ != f.ptime_)
    return false;
  return true;
}

inline
Encoding preferred(Media::Formats const& p) {
  for(unsigned i = 0; i < sizeof(p.mask_)*8; ++i) {
    if((1 << i) & p.mask_) return Encoding(i);
  }
  assert(0);
  return Encoding(0);
}

struct Payload;
void intrusive_ptr_add_ref(Payload*);
void intrusive_ptr_release(Payload*);

struct Packet {
  Packet();
  Packet(const void* srcid,
    Encoding e,
    posix_time::ptime const& ts,
    posix_time::time_duration const& duration,
    boost::intrusive_ptr<Payload> const& payload = 0) 
  :
    srcid_(srcid),
    encoding_(e),
    ts_(ts),
    duration_(duration),
    payload_(payload) 
  {}

  const void* srcid_;
  Encoding encoding_;
  posix_time::ptime ts_;
  posix_time::time_duration duration_;
  boost::intrusive_ptr<Payload> payload_;
};

typedef boost::function<void (Packet const& packet)> Sink;
typedef boost::function<void (Sink const& sink)> Source;
typedef boost::function<void (posix_time::ptime const& ptime, boost::function<void(posix_time::ptime const& )> const&)> Clock;

Clock system_clock();

typedef std::pair<Source, Sink> Filter;

Filter identity_filter();

Filter jitter(Clock const& clock = system_clock());

Filter pcma_decoder();
Filter pcma_encoder();

Filter pcmu_decoder();
Filter pcmu_encoder();

Filter ilbc_decoder();
Filter ilbc_encoder();


Filter encoder(Encoding e);
Filter decoder(Encoding e);

Filter packetizer(Packetization ptime);

Source on_change_encoding(Source source, boost::function<void (Encoding)>);


Sink split(Sink const&, Sink const&);
Sink mixer(Sink const&);


struct Adaptor;
void intrusive_ptr_add_ref(Adaptor*);
void intrusive_ptr_release(Adaptor*);

boost::intrusive_ptr<Adaptor> make_adaptor();

Sink get_sink(boost::intrusive_ptr<Adaptor> const&);
void set_sinks(boost::intrusive_ptr<Adaptor> const&, std::list<std::pair<Sink, Formats> >& list);


struct Rtp;
void intrusive_ptr_add_ref(Rtp*);
void intrusive_ptr_release(Rtp*);

boost::intrusive_ptr<Rtp> make_rtp();

void rtp_set_encoding_map(boost::intrusive_ptr<Rtp> rtp, int pcma, int pcmu, int ilbc);

Source get_source(boost::intrusive_ptr<Rtp> const& rtp);
Sink get_sink(boost::intrusive_ptr<Rtp> const& rtp);

void set_rtp_map(boost::intrusive_ptr<Rtp> const& rtp, boost::function<int (Encoding)> const& map);
void set_rtp_map(boost::intrusive_ptr<Rtp> const& rtp, boost::function<Encoding (int)> const& map);

void set_local_endpoint(boost::intrusive_ptr<Rtp> const& rtp, asio::ip::udp::endpoint const& ep);
asio::ip::udp::endpoint get_local_endpoint(boost::intrusive_ptr<Rtp> const& rtp);

void set_remote_endpoint(boost::intrusive_ptr<Rtp> const& rtp, asio::ip::udp::endpoint const& ep);


struct FileSource;
void intrusive_ptr_add_ref(FileSource*);
void intrusive_ptr_release(FileSource*);

boost::intrusive_ptr<FileSource> make_file_source(const char* name, Format const& fmt, boost::function<void ()> const& eof, Clock const& clock = system_clock());

Source get_source(boost::intrusive_ptr<FileSource> const& fs);


struct FileSink;
void intrusive_ptr_add_ref(FileSink*);
void intrusive_ptr_release(FileSink*);

boost::intrusive_ptr<FileSink> make_file_sink(const char* name, Encoding enc);


void start();
void stop();

}

