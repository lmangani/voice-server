#include "json.hpp"
#include "media.hpp"

#include <boost/function.hpp>
#include <boost/mpl/list.hpp>
#include <boost/mpl/joint_view.hpp>
#include <boost/mpl/transform_view.hpp>
#include <boost/mpl/copy.hpp>
#include <boost/any.hpp>
#include <boost/bind/make_adaptable.hpp>

typedef boost::mpl::list<Media::Audio::LinearFrame<80> 
  ,Media::Audio::AlawFrame<160>
  ,Media::Audio::AlawFrame<240>
  ,Media::Audio::UlawFrame<160>
  ,Media::Audio::UlawFrame<240>
  ,Media::Audio::IlbcFrame<160>
  ,Media::Audio::IlbcFrame<240>
> AudioCodecs;


namespace Media { namespace Audio {
static const char* codec_name(LinearFrame<80>*) { return "L16/8000;p=10";}
static const char* codec_name(AlawFrame<160>*) { return "PCMA/8000;p=20";}
static const char* codec_name(AlawFrame<240>*) { return "PCMA/8000;p=30";}
static const char* codec_name(UlawFrame<160>*) { return "PCMU/8000;p=20";}
static const char* codec_name(UlawFrame<240>*) { return "PCMU/8000;p=30";}
static const char* codec_name(IlbcFrame<160>*) { return "iLBC/8000;p=20";}
static const char* codec_name(IlbcFrame<240>*) { return "iLBC/8000;p=30";}
}}

struct ForCodecName {
  template<typename F>
  static void exec(boost::mpl::end<AudioCodecs>::type*, boost::mpl::end<AudioCodecs>::type*, std::string const& name, F f) {
    throw std::runtime_error("unknown codec");
  }

  template<typename I, typename F>
  static void exec(I*, boost::mpl::end<AudioCodecs>::type*, std::string const& name, F f) {
    typedef typename boost::mpl::deref<I>::type type;
    if(name == codec_name((type*)0)) {
      f((type*)0);
    }
    else
      exec((typename boost::mpl::next<I>::type*)0, (boost::mpl::end<AudioCodecs>::type*)0, name, f);
  }
};

template<typename F>
void for_codec_name(std::string const& name, F f) {
  ForCodecName::exec((boost::mpl::begin<AudioCodecs>::type*)0, (boost::mpl::end<AudioCodecs>::type*)0, name, f);
}

typedef boost::make_variant_over<AudioCodecs> VarFrame;

template<typename A>
struct DeVar {
  void operator()(VarFrame const& f) {
    boost::apply_visitor(f, boost::bind(boost::ref(a_), _1));
  }

  A a_;
};


typedef std::map<std::string, boost::any> Objects;
Objects g_objects;


template<template<typename T> class S>
struct Templatify {
  template<typename T>
  struct apply {
      typedef S<T> type;
  };
};

template<typename T>
struct Sink {
  Sink(boost::function<void (T const& t)> const& f = boost::function<void (T const& t)>()) : p_(boost::make_shared<boost::function<void (T const& t)>>(f)) {}

  void operator()(T const& t) {
    if(!p_->empty()) (*p_)(t);
  }

  boost::shared_ptr<boost::function<void (T const&)> > p_;
};

typedef boost::make_variant_over<
  boost::mpl::transform<
    boost::mpl::copy<
      boost::mpl::copy<
        AudioCodecs,
        boost::mpl::front_inserter<
          boost::mpl::transform<
            AudioCodecs,
            Templatify<Media::Audio::Packet>
          >::type
        >
      >::type,
      boost::mpl::front_inserter<
        boost::mpl::list<
          Media::Transport::UdpPacket,
          Media::Transport::RtpPacket>
      >
    >::type,
    Templatify<Sink>>::type
  >::type VSink; 


VSink& get_sink(JSON::Value const& id) {
  Objects::iterator i = g_objects.find(boost::get<JSON::String>(id));

  if(i == g_objects.end())
    throw std::range_error("sink");

  return boost::any_cast<VSink&>(i->second);
}

template<typename T>
Sink<T>& insert_sink(JSON::Value const& id) {
  return boost::get<Sink<T> >(boost::any_cast<VSink&>(g_objects.insert(std::make_pair(boost::get<JSON::String>(id), Sink<T>())).first->second));
}

template<typename T>
Sink<T>& set_sink(Sink<T>& v, boost::function<void (T const&)> const& f) {
  typedef boost::function<void (T const&)> F;  
  Media::post(boost::bind(static_cast<F& (F::*)(F const&)>(&F::operator=), v.p_, f)); // we have to perform assignment in media thread
  return v;
}

template<typename T>
Sink<T>& set_sink(JSON::Value const& id, boost::function<void (T const&)> const& f) {
  return set_sink(insert_sink<T>(id), f);
}

template<typename T>
struct Source {
  Source() : p_(boost::make_shared<boost::function<std::pair<T, size_t> (size_t)>>()) {}

  std::pair<T, size_t> operator()(size_t n) const {
    return p_->empty() ? std::make_pair(T(),0ul) : (*p_)(n);
  }
  
  Media::Audio::PullBranch<Source<T>> branch() const {
    return *this;
  }

  boost::shared_ptr<boost::function<std::pair<T,size_t> (size_t)>> p_;
};

typedef boost::make_variant_over<boost::mpl::transform<AudioCodecs, Templatify<Source>>::type>::type VSource;

VSource& get_source(JSON::Value const& v) {
  Objects::iterator i = g_objects.find(boost::get<JSON::String>(v));
  if(g_objects.end() == i)
    throw std::range_error("non existing source name " + boost::get<JSON::String>(v));

  return boost::any_cast<VSource&>(i->second);
}

template<typename T>
Source<T>& insert_source(JSON::Value const& v) {
  return boost::get<Source<T>>(boost::any_cast<VSource&>(g_objects.insert(std::make_pair(boost::get<JSON::String>(v), Source<T>())).first->second));
}

template<typename T, typename S>
Source<T>& set_source(JSON::Value const& v, S const& s) {
  Source<T>& t = insert_source<T>(v);
  
  typedef boost::function<std::pair<T,size_t> (size_t)> F;
  Media::post(boost::bind(static_cast<F& (F::*)(F const&)>(&F::operator=), t.p_, Media::Audio::PullSplitter<S>(s)));
  
  return t;
}


struct make_jitter_buffer {
  template<typename T>
  void operator()(JSON::Value const& name, T*) {
    auto buffer = boost::make_shared<Media::Audio::JitterBuffer<T> >();

    set_sink<Media::Audio::Packet<T>>(name, boost::bind(&Media::Audio::JitterBuffer<T>::push, buffer, _1));
    set_source<T>(name, boost::bind(&Media::Audio::JitterBuffer<T>::pull, buffer));
  }
};

JSON::Value create_jitter_buffer(JSON::Array& array) {
  for_codec_name(boost::get<JSON::String>(array[0]), boost::bind<void>(make_jitter_buffer(), array[1], _1));
  return JSON::null;
}


struct make_clock {
  typedef boost::any result_type;
  template<typename T>
  boost::any operator()(Source<T> const& source, Sink<T> const& sink) const {
    return Media::Audio::clock(source.branch(), sink);
  }

  template<typename T1, typename T2>
  boost::any operator()(T1, T2) const { throw std::runtime_error("type error"); }
};

JSON::Value create_clock(JSON::Array& array) {
  std::string name = boost::get<JSON::String>(array[0]);
  g_objects[name] = boost::apply_visitor(make_clock(),get_source(array[1]),  get_sink(array[2]));
  return JSON::null;
}


struct Unidecode {
  template<typename T>
  void operator()(T const& t) {
    if(&push_.type() != &typeid(boost::shared_ptr<Media::Audio::JitterBuffer<T> >)) {
      boost::shared_ptr<Media::Audio::JitterBuffer<T> > buffer = boost::make_shared<Media::Audio::JitterBuffer<T> >();
      pull_ = Media::Audio::resize<80>(decode(boost::bind(&Media::Audio::JitterBuffer<T>::pull, buffer)));
      push_ = buffer;
    }
    
    boost::any_cast<boost::shared_ptr<Media::Audio::JitterBuffer<T> > >(push_)->push(t);
  }

  Media::Audio::LinearFrame<80> operator()() {
    return pull_();
  }

  boost::any push_;
  boost::function<Media::Audio::LinearFrame<80> ()> pull_;
};

struct make_decode {
  template<typename T>
  void operator()(JSON::Value const& name, Source<T> const& t) const {
    using namespace Media::Audio;
    set_source<LinearFrame<80>>(name, resize<80>(decode(t.branch())));
  }

  void operator()(JSON::Value const&, Source<Media::Audio::LinearFrame<80> > const&) const {
    throw std::runtime_error("can't decode linear");
  }
};

void decode(JSON::Array& array) {
  boost::apply_visitor(boost::bind<void>(make_decode(), array[0], _1), get_source(array[1]));
}


struct make_encode {
  template<typename T>
  void operator()(JSON::Value const& from, JSON::Value const& to, T*) const {
    using namespace Media::Audio;
    set_source<T>(to, encode<T>(resize<T::duration>(boost::get<Source<LinearFrame<80>>>(get_source(from)).branch())));
  }

  void operator()(JSON::Value const&, JSON::Value const&, Media::Audio::LinearFrame<80>*) const {
    throw std::runtime_error("can't encode linear");
  }
};

void encode(JSON::Array& array) {
  for_codec_name(boost::get<JSON::String>(array[0]), boost::bind<void>(make_encode(), array[1], array[2], _1));
}


struct make_packetize {
  template<typename T>
  void operator()(JSON::Value const& name, Sink<T> const& s) const {
    set_sink<T>(name, Media::Audio::Packetizer<Sink<T> >(s));
  }

  void operator()(JSON::Value const&, Sink<Media::Transport::UdpPacket> const&) const {
    throw std::logic_error("type error");
  }

  void operator()(JSON::Value const&, Sink<Media::Transport::RtpPacket> const&) const {
    throw std::logic_error("type error");
  }

  template<typename T>
  void operator()(JSON::Value const&, Sink<Media::Audio::Packet<T> > const&) const {
    throw std::logic_error("type error");
  }
};
  
JSON::Value packetize(JSON::Array& array) {
  boost::apply_visitor(boost::bind<void>(make_packetize(), array[1], _1), get_sink(array[0]));
  return JSON::null;
}


JSON::Value mix(JSON::Array& array) {
  using namespace Media::Audio;

  std::vector<decltype(Source<LinearFrame<80>>().branch())> m;

  for(size_t i = 1; i < array.size(); ++i)
    m.push_back(boost::get<Source<LinearFrame<80>>>(get_source(array[i])).branch());
  
  set_source<LinearFrame<80> >(array[0], Mixer<decltype(m)>(m));

  return JSON::null;
}


struct make_splitter {
  template<typename T>
  void operator()(JSON::Array& args, Sink<T> const&) const {
    std::vector<Sink<T>> s;
    
    for(size_t i = 0; i < args.size(); ++i) 
      s.push_back(insert_sink<T>(args[i]));
    
    set_sink<T>(args[0], Media::Audio::Splitter<decltype(s)>(s));
  }
};

JSON::Value split(JSON::Array& array) {
  boost::apply_visitor(boost::bind<void>(make_splitter(), boost::ref(array), _1), get_sink(array[0])); 
  return JSON::null;
}


boost::asio::ip::udp::endpoint endpoint(JSON::Value& v) {
  JSON::Object& o = boost::get<JSON::Object>(v);

  return boost::asio::ip::udp::endpoint(
    boost::asio::ip::address::from_string(boost::get<JSON::String>(o["address"])),
    boost::get<JSON::Number>(o["port"]));
}

JSON::Value create_socket(JSON::Array& array) {
  Media::Transport::Socket s;

  s.socket_->open(boost::asio::ip::udp::v4());

  set_sink<Media::Transport::UdpPacket>(array[0], s);
  s.recv(insert_sink<Media::Transport::UdpPacket>(array[1]));

  if(array.size() >= 4) {
    s.socket_->bind(endpoint(array[2]));
    s.socket_->connect(endpoint(array[3]));
  }
  else {
    s.socket_->connect(endpoint(array[2]));
  }

  return JSON::null;
}

struct make_reader {
  template<typename T>
  void operator()(JSON::Value const& name, std::string const& filename, T*) const {
    int fd = open(filename.c_str(), O_RDONLY);
    if(-1 == fd)
      throw std::runtime_error("not found");

    set_source<T>(name, Media::Transport::Reader<T>(fd));
  }
};

void create_reader(JSON::Array& array) {
  for_codec_name(boost::get<JSON::String>(array[2]), boost::bind<void>(make_reader(), array[0], boost::get<JSON::String>(array[1]), _1));
}

struct make_writer {
  template<typename T>
  void operator()(JSON::Value const& name, std::string const& filename, T*) const {
    int fd = open(filename.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0666);
    if(-1 == fd)
      throw std::runtime_error("not found");

    set_sink<T>(name, Media::Transport::Writer<>(fd));
  }
};

JSON::Value create_write(JSON::Array& array) {
  for_codec_name(boost::get<JSON::String>(array[2]), boost::bind<void>(make_writer(), array[0], boost::get<JSON::String>(array[1]), _1));
  return JSON::null;
}


JSON::Value rtp2packet(JSON::Array& array) {
  struct make_rtp2packet {
    template<typename T>
    void operator()(JSON::Value const& from, JSON::Value const& to, JSON::Value const& pt, T*) {
      typedef Media::Audio::Packet<T> Packet;
      using namespace Media::Transport;
      set_sink<UdpPacket>(from, make_parse<RtpPacket>(rtp2packet<Packet>(boost::get<JSON::Number>(pt),insert_sink<Packet>(to))));
    }
  };

  for_codec_name(boost::get<JSON::String>(array[2]), boost::bind<void>(make_rtp2packet(), array[0], array[1], array[3], _1)); 
  return JSON::null;
}

JSON::Value packet2rtp(JSON::Array& array) {
  struct make_packet2rtp {
    template<typename T>
    void operator()(JSON::Value const& from, JSON::Value const& to, JSON::Value const& pt, T*) {
      using namespace Media::Transport;
      typedef Media::Audio::Packet<T> Packet;

      set_sink<Packet>(from, packet2rtp<Packet>(boost::get<JSON::Number>(pt), make_parse<UdpPacket>(insert_sink<UdpPacket>(to))));
    }  
  };
 
  for_codec_name(boost::get<JSON::String>(array[2]), boost::bind<void>(make_packet2rtp(), array[0], array[1], array[3], _1));
  return JSON::null;
}


template<typename T>
boost::weak_ptr<T> make_weak(boost::shared_ptr<T> const& t) {
  return boost::weak_ptr<T>(t);
}

JSON::Value condition(JSON::Array& array) {
  struct Condition {
    template<typename T, typename D>
    void operator()(T const&, boost::weak_ptr<D> const& from, boost::weak_ptr<D> const& to) {
      boost::shared_ptr<D> f(from), t(to);
      if(f && t) *t = *f;
    }
  };

  struct make_condition {
    typedef void result_type;

    template<typename T, typename T2>
    void operator()(Sink<T>& s, Sink<T2> const& from, JSON::Value const& to) const {
      set_sink(s, boost::function<void (T const&)>(boost::bind<void>(Condition(), _1, make_weak(from.p_), make_weak(insert_sink<T2>(to).p_))));
    }

    template<typename T, typename T2>
    void operator()(Sink<T> const& s, Source<T2> const& from, JSON::Value const& to) const {
      //set_sink(s, boost::bind<void>(Condition(), _1, make_weak(from.p_), make_weak(insert_source<T2>(to).p_)));  
    }
  };

  Objects::iterator i = g_objects.find(boost::get<JSON::String>(array[1]));

  if(g_objects.end() == i) throw std::range_error("name not found");
  if(&i->second.type() == &typeid(VSink)) {
    boost::apply_visitor(boost::bind(make_condition(), _1, _2, array[2]), get_sink(array[0]), boost::any_cast<VSink&>(i->second));
  }
  else if(&i->second.type() == &typeid(VSource)) {
    boost::apply_visitor(boost::bind(make_condition(), _1, _2, array[2]), get_sink(array[0]), boost::any_cast<VSource const&>(i->second));
  }
  else
    throw std::logic_error("type error");

  return JSON::null;  
}

JSON::Value destroy(JSON::Array& args) {
  g_objects.erase(boost::get<JSON::String>(args[0]));
  return JSON::null;
}


typedef std::map<std::string, boost::function<JSON::Value (JSON::Array& )> > MethodsMap;
MethodsMap g_methods;

void request(JSON::Object& v) {
  try {
    MethodsMap::iterator i = g_methods.find(boost::get<JSON::String>(v["method"]));

    if(i == g_methods.end())
      throw std::runtime_error("Unknown Method");
    
    if(!(v["id"] == JSON::null)) {
      std::ostream::sentry sentry(std::cout); 
      JSON::Value r = i->second(boost::get<JSON::Array>(v["params"]));
      std::cout << "{\"result\":" << r << ",\"error\":null,\"id\":" << v["id"] << "}" << std::endl;
    }
  }
  catch(std::exception const& e) {
    if(!(v["id"] == JSON::null)) {
      std::ostream::sentry sentry(std::cout);
      std::cout << "{\"result\":null,\"error\":{\"message\":\"" << e.what() << "\"},\"id\":" << v["id"] << "}" << std::endl; 
    }
  }
}


int main(int argc, char* argv[]) {
  g_methods["delete"] = destroy;
  g_methods["socket"] = create_socket;
  g_methods["rtp2packet"] = rtp2packet; 
  g_methods["packet2rtp"] = packet2rtp;
  g_methods["write"] = create_write;
  g_methods["clock"] = create_clock;
  g_methods["jitter"] = create_jitter_buffer;
  g_methods["packetize"] = packetize;
  g_methods["mix"] = mix;
  g_methods["split"] = split;
  g_methods["condition"] = condition;  

  Media::start();  

  while(std::cin) {
    JSON::Value v = JSON::parse(std::cin);
    if(!(v == JSON::null)) {
      request(boost::get<JSON::Object>(v));
    }
  }

  Media::stop();

  return 0;
}

