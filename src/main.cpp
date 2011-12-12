#include "json.hpp"
#include "media.hpp"

#include <boost/mpl/list.hpp>
#include <boost/mpl/joint_view.hpp>
#include <boost/mpl/transform_view.hpp>
#include <boost/mpl/copy.hpp>
#include <boost/any.hpp>

using namespace std::placeholders;

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

typedef std::map<std::string, boost::any> Objects;
Objects g_objects;

template<typename T>
struct Source : Media::SharedFunctor<Media::Root<Media::DelayedSource<T>>> {
  typedef T source_type;

  template<typename... Args>
  Source(Args&&... args) : Media::SharedFunctor<Media::Root<Media::DelayedSource<T>>>(std::forward<Args>(args)...) {}
  Source() : Media::SharedFunctor<Media::Root<Media::DelayedSource<T>>>(Media::SharedFunctor<Media::Root<Media::DelayedSource<T>>>::make(Media::DelayedSource<T>())) {}
};

template<template<typename T> class S>
struct Templatify {
  template<typename T>
  struct apply {
      typedef S<T> type;
  };
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
          Media::Transport::RtpPacket,
          std::pair<boost::asio::ip::udp::endpoint, Media::Transport::UdpPacket>
        >
      >
    >::type,
    Templatify<Source>>::type
  >::type VSource; 

template<typename T>
Source<T> get_source(JSON::Value const& id) {
  return boost::get<Source<T>>(boost::any_cast<VSource&>(g_objects.insert(std::make_pair(boost::get<JSON::String>(id), Source<T>())).first->second));
}

template<typename T, typename A>
void set_source(Source<T> v, A&& a) {
  Media::SharedFunctor<A> sa = Media::SharedFunctor<A>::make(std::forward<A>(a));
  Media::g_io.post([v, sa]() mutable {v.p_->a_ = sa;});
}

template<typename T, typename A>
void set_source(JSON::Value const& id, A&& a) {
  set_source(get_source<T>(id), std::forward<A>(a));
}

struct MakeJitter {
  template<typename T>
  void operator()(JSON::Array& args, T*) {
    using namespace Media::Audio;
    auto to = get_source<T>(args[1]);
    auto from = get_source<Packet<T>>(args[2]);
    Media::g_io.post([=]() mutable { set_source(to,rate(jitter(branch(std::move(from))))); });
    //Media::g_io.post([=]() mutable { set_source(to, jitter(branch(std::move(from)))); });
  }
};

JSON::Value jitter(JSON::Array& args) {
  for_codec_name(boost::get<JSON::String>(args[0]), std::bind(MakeJitter(), std::ref(args), _1));
  return JSON::null;
}


struct MakeDecode {
  template<typename T>
  void operator()(JSON::Array& args, T*) {
    using namespace Media::Audio;
    set_source<LinearFrame<80>>(args[1], resize<80>(Media::Audio::decode(branch(get_source<T>(args[2])))));
  }

  void operator()(JSON::Array& args, Media::Audio::LinearFrame<80>*) {
    throw std::logic_error("can't decode linear source");
  }
};

JSON::Value decode(JSON::Array& args) {
  for_codec_name(boost::get<JSON::String>(args[0]), std::bind(MakeDecode(), std::ref(args), _1));
  return JSON::null;
}


struct MakeEncode {
  template<typename T>
  void operator()(JSON::Array& args, T*) {
    using namespace Media::Audio;
    set_source<T>(args[1], encode<T>(resize<duration(*(T*)0)>(branch(std::move(get_source<LinearFrame<80>>(args[2]))))));
  }

  void operator()(JSON::Array& args, Media::Audio::LinearFrame<80>*) {
    throw std::logic_error("can't encode to linear");
  }
};

JSON::Value encode(JSON::Array& args) {
  for_codec_name(boost::get<JSON::String>(args[0]), std::bind(MakeEncode(), std::ref(args), _1));
  return JSON::null;
}


JSON::Value mix(JSON::Array& args) {
  using namespace Media::Audio;

  std::vector<decltype(Media::branch(Source<LinearFrame<80>>()))> a;
  for(auto i: boost::get<JSON::Array>(args[1])) {
    a.push_back(Media::branch(std::move(get_source<LinearFrame<80>>(i))));
  }

  set_source<LinearFrame<80>>(args[0], mix(a));
  return JSON::null;
}


struct MakeTimestamp {
  template<typename T>
  void operator()(JSON::Array& args, T*) {
    using namespace Media::Audio;
    set_source<Packet<T>>(args[1], Media::transform_to<Packet<T>>(branch(get_source<T>(args[2])), rand()));
  }
};

JSON::Value timestamp(JSON::Array& args) {
  for_codec_name(boost::get<JSON::String>(args[0]), std::bind(MakeTimestamp(), std::ref(args), _1));
  return JSON::null;
}

struct MakePack {
  template<typename T>
  void operator()(JSON::Array& args, T*) {
    using namespace Media::Transport;
    set_source<RtpPacket>(args[1], Media::transform_to<RtpPacket>(branch(get_source<Media::Audio::Packet<T>>(args[2])), boost::get<JSON::Number>(args[3])));
  }
};

JSON::Value pack(JSON::Array& args) {
  for_codec_name(boost::get<JSON::String>(args[0]), std::bind(MakePack(), std::ref(args), _1));
  return JSON::null;
}


struct MakeUnpack {
  template<typename T>
  void operator()(JSON::Array& args, T*) {
    using namespace Media::Audio;
    set_source<Packet<T>>(args[1], Media::transform_to<Packet<T>>(branch(std::move(get_source<Media::Transport::RtpPacket>(args[2]))), boost::get<JSON::Number>(args[3])));
  }
};

JSON::Value unpack(JSON::Array& args) {
  for_codec_name(boost::get<JSON::String>(args[0]), std::bind(MakeUnpack(), std::ref(args), _1));
  return JSON::null;
}

struct MakeCondition {
  struct Visitor {
    typedef void result_type;

    template<typename T>
    void operator()(Source<T> f, Source<T> t) const {
      set_source(t, branch(std::move(f)));
    }

    template<typename T1, typename T2>
    void operator()(T1 const&, T2 const&) const {}
  };

  template<typename T>
  void operator()(JSON::Array& args, T*) {
    Objects::iterator i = g_objects.find(boost::get<JSON::String>(args[3]));
    if(i == g_objects.end())
      throw std::range_error("not found");

    VSource f = boost::any_cast<VSource>(i->second);

    i = g_objects.find(boost::get<JSON::String>(args[4]));
    if(i == g_objects.end())
      throw std::range_error("not found");

    VSource t = boost::any_cast<VSource>(i->second);

    if(t.which() != f.which())
      throw std::logic_error("incompatible source types");
    
    g_objects[boost::get<JSON::String>(args[1])] = condition(branch(get_source<T>(args[2])), [=]{ boost::apply_visitor(Visitor(), f, t);});
  }
};

JSON::Value condition(JSON::Array& args) {
  for_codec_name(boost::get<JSON::String>(args[0]), std::bind(MakeCondition(), std::ref(args), _1));
  return JSON::null;
}

JSON::Value event(JSON::Array& args) {
  return JSON::null;
}

JSON::Value tag(JSON::Array& args) {
  return JSON::null;
}

JSON::Value socket(JSON::Array& args) {
  using namespace Media::Transport;

  Socket s;
  //s.s_.open();
  s.s_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(boost::get<JSON::String>(args[3])), boost::get<JSON::Number>(args[4])));
  
  auto sk = get_source<std::pair<boost::asio::ip::udp::endpoint, UdpPacket>>(args[1]);
  auto src = get_source<UdpPacket>(args[0]);
  auto sock = std::make_shared<Socket>(std::move(s));
  
  Media::g_io.post([=]() mutable { set_source(src, push(branch(std::move(sk)), std::move(*sock))); });
  
  return JSON::null;
}

struct MakeReader {
  template<typename T>
  void operator()(JSON::Array& args, T*) {
    int fd = open(boost::get<JSON::String>(args[2]).c_str(), O_RDONLY);

    if(fd == -1) throw boost::system::system_error(boost::system::errc::make_error_code(boost::system::errc::errc_t(errno)));

    set_source<T>(args[1], Media::Transport::Reader<T>(fd));
  }
};

JSON::Value reader(JSON::Array& args) {
  for_codec_name(boost::get<JSON::String>(args[0]), std::bind(MakeReader(), std::ref(args), _1)); 
  return JSON::null;
}


struct MakeWriter {
  template<typename T>
  void operator()(JSON::Array& args, T*) {
    int fd = open(boost::get<JSON::String>(args[3]).c_str(), O_WRONLY);    

    if(fd == -1) throw boost::system::system_error(boost::system::errc::make_error_code(boost::system::errc::errc_t(errno))); 
    
    Source<T> src = get_source<T>(args[2]);
    
    auto a = std::make_shared<boost::any>();
    g_objects[boost::get<JSON::String>(args[1])] = a;

    Media::g_io.post([=]() mutable { *a = move_to_shared(push(branch(std::move(src)), Media::Transport::Writer(fd)));});
  }
};

JSON::Value writer(JSON::Array& args) {
  for_codec_name(boost::get<JSON::String>(args[0]), std::bind(MakeWriter(), std::ref(args), _1));
  return JSON::null;
}

JSON::Value destroy(JSON::Array& args) {
  Objects::iterator i = g_objects.find(boost::get<JSON::String>(args[0]));

  if(i == g_objects.end()) throw std::range_error("not found");
  
  boost::any x = i->second;
  
  g_objects.erase(i);

  Media::g_io.post([=]() mutable { 
    boost::any a;
    std::swap(x, a);
  });

  return JSON::null;
}


typedef std::map<std::string, std::function<JSON::Value (JSON::Array& )> > MethodsMap;
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

  g_methods["jitter"] = jitter;
  g_methods["decode"] = decode;
  g_methods["encode"] = encode;
  g_methods["mix"] = mix;
  g_methods["timestamp"] = timestamp;
  g_methods["pack"] = pack;
  g_methods["unpack"] = unpack;
  g_methods["condition"] = condition;
  g_methods["tag"] = tag;
  g_methods["event"] = event;
  g_methods["socket"] = static_cast<JSON::Value (*)(JSON::Array&)>(socket);
  g_methods["reader"] = reader;
  g_methods["writer"] = writer;

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

