#include <iostream>
#include "media.hpp"
#include "json.hpp"
#include <boost/thread.hpp>

boost::asio::io_service the_event_io;

template<typename C>
struct EventCallback {
  EventCallback(C const& c) : c_(c) {}

  void operator()() {
    the_event_io.post(c_);
  }

  template<typename A>
  void operator()(A a) {
    the_event_io.post(boost::bind(c_, a));
  }

  template<typename A1, typename A2>
  void operator()(A1 a1, A2 a2) {
    the_event_io.post(boost::bind(c_, a1, a2));
  }

  C c_;
};

template<typename C>
EventCallback<C> wrap_event_callback(C const& c) {
  return EventCallback<C>(c);
}

void start_event_thread() {
  boost::thread thread(boost::bind(&boost::asio::io_service::run,&the_event_io));
}

void stop_event_thread() {
  the_event_io.stop();
}

Media::PayloadType irtpmap(int e, int alaw, int ulaw, int te) {
  if(e == alaw)
    return Media::G711A;
  else if(e == ulaw) 
    return Media::G711U;
  else if(e == te)
    return Media::RFC2833;

  return 0;
}

int srtpmap(Media::PayloadType pt, int alaw, int ulaw, int te) {
  if(pt == Media::G711A)
    return alaw;
  else if(pt == Media::G711U) 
    return ulaw;
  else if(pt == Media::RFC2833)
    return te;

  return -1;
}
 
struct Resource {
  Media::PullSink pull_sink_;
  Media::Source pull_source_;
  
  Media::Sink   push_sink_;
  Media::PushSource push_source_;

  boost::function<void()> close_;

  boost::function<JSON::Value (JSON::Array&)> configure_;
};

typedef std::map<std::string, Resource> ResourceMap;
ResourceMap g_resource_map;

template<typename T>
T get_decoder(Media::PayloadType pt, T const& t) {
  if(pt == Media::G711A)
    return Media::decoder<Media::G711aDecoder>(t);
  else if(pt == Media::G711U)
    return Media::decoder<Media::G711uDecoder>(t);

  throw std::runtime_error("Unknown Decoder");
}

template<typename T>
T get_encoder(Media::PayloadType pt, T const& t) {
  if(pt == Media::G711A)
    return Media::encoder<Media::G711aEncoder>(t);
  else if(pt == Media::G711U)
    return Media::encoder<Media::G711uEncoder>(t);

  throw std::runtime_error("Unknown Encoder");
}

Media::PayloadType get_payload_type(std::string const& name) {
  for(size_t i = 0; i != sizeof(Media::Payloads)/sizeof(*Media::Payloads); ++i)
    if(name == Media::Payloads[i]->name())
      return Media::Payloads[i];

  throw std::runtime_error("unknown codec");
}

Media::Source get_source(JSON::Value& v) {
  if(boost::get<JSON::String>(&v)) {
    ResourceMap::iterator i = g_resource_map.find(boost::get<JSON::String>(v));

    if(i == g_resource_map.end() || i->second.pull_source_.empty())
      throw std::runtime_error("not a pull source");

    return i->second.pull_source_;
  }
  else if(boost::get<JSON::Object>(&v)) {
    JSON::Object& m = boost::get<JSON::Object>(v);
    if(!(m["decode"] == JSON::null)) 
      return get_decoder(get_payload_type(boost::get<JSON::String>(m["codec"])), get_source(m["decode"]));
    else if(!(m["encode"] == JSON::null)) 
      return get_encoder(get_payload_type(boost::get<JSON::String>(m["codec"])), get_source(m["encode"]));
    else 
      throw std::runtime_error("Unsupported Transformation");
  }
  else if(boost::get<JSON::Array>(&v)) {
    JSON::Array a;
    if(a.size() == 0)
      return Media::Source();

    Media::Source s = get_source(a[0]);

    for(size_t i = 1; i != a.size(); ++i)
      s = Media::mix2(s, get_source(a[i]));

    return s;
  }
  else
    throw std::runtime_error("source can be source id or transformation object");
}

Media::Sink get_sink(JSON::Value& v) {
  if(boost::get<JSON::String>(&v)) {
    ResourceMap::iterator i = g_resource_map.find(boost::get<JSON::String>(v));

    if(i == g_resource_map.end() || i->second.push_sink_.empty())
      throw std::runtime_error("not a push sink");

    return i->second.push_sink_;
  }
  else if(boost::get<JSON::Object>(&v)) {
    JSON::Object& m = boost::get<JSON::Object>(v);
    if(!(m["decode"] == JSON::null))
      return get_decoder(get_payload_type(boost::get<JSON::String>(m["codec"])), get_sink(m["decode"]));
    else if(!(m["encode"] == JSON::null))
      return get_encoder(get_payload_type(boost::get<JSON::String>(m["codec"])), get_sink(m["encode"]));
    else 
      throw std::runtime_error("Unsupported Transformation");
  }
  else if(boost::get<JSON::Array>(&v)) {
    JSON::Array a;
    if(a.size() == 0)
      throw std::runtime_error("splitter can't have empty sink list");

    Media::Sink s = get_sink(a[0]);

    for(size_t i = 1; i != a.size(); ++i)
      s = Media::split2(s, get_sink(a[i]));

    return s;
  }
  else {
    throw std::runtime_error("sink can be sink id or transformation object");
  }
}

typedef std::map<std::string, boost::function<JSON::Value (JSON::Array& )> > MethodsMap;
MethodsMap g_methods;

MethodsMap g_constructors;

JSON::Value create(JSON::Array& params) {
  MethodsMap::iterator i = g_constructors.find(boost::get<JSON::String>(params.at(0)));

  if(i == g_constructors.end())
    throw std::runtime_error("Unknown Constructor");

  return i->second(params);  
}

void request(JSON::Object& v) {
  try {
    MethodsMap::iterator i = g_methods.find(boost::get<JSON::String>(v["method"]));

    if(i == g_methods.end())
      throw std::runtime_error("Unknown Method");
    
    if(!(v["id"] == JSON::null)) {
      std::ostream::sentry sentry(std::cout); 
      std::cout << "{\"result\":" << i->second(boost::get<JSON::Array>(v["params"])) << ",\"error\":null,\"id\":" << v["id"] << "}" << std::endl;
    }
  }
  catch(std::exception const& e) {
    if(!(v["id"] == JSON::null)) {
      std::ostream::sentry sentry(std::cout);
      std::cout << "{\"result\":null,\"error\":{\"message\":\"" << e.what() << "\"},\"id\":" << v["id"] << "}" << std::endl; 
    }
  }
}

JSON::Value rtp_configure(boost::intrusive_ptr<Media::Rtp> rtp, JSON::Array& v) {
  JSON::Object& a = boost::get<JSON::Object>(v.at(1));

  JSON::Object::iterator i = a.find("rtpmap");
  if(i != a.end()) {
    JSON::Object& rtpmap = boost::get<JSON::Object>(i->second);

    int alaw = boost::get<double>(rtpmap[Media::G711A->name()]);
    int ulaw = boost::get<double>(rtpmap[Media::G711U->name()]);
    int te = boost::get<double>(rtpmap[Media::RFC2833->name()]);

    rtp->set_payload_type_to_rtp_type(boost::bind(srtpmap, _1, alaw, ulaw, te));
    rtp->set_rtp_type_to_payload_type(boost::bind(irtpmap, _1, alaw, ulaw, te));
  }
  
  i = a.find("remote");
  if(i != a.end()) {
    JSON::Object& ep = boost::get<JSON::Object>(i->second);
    rtp->connect(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(boost::get<JSON::String>(ep["address"])), boost::get<double>(ep["port"])));
  }

  i = a.find("local");
  if(i != a.end()) {
    JSON::Object& ep = boost::get<JSON::Object>(i->second);
    rtp->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(boost::get<JSON::String>(ep["address"])), boost::get<double>(ep["port"])));
  }

  return JSON::make_object("local",JSON::make_object(
      "address", rtp->local_endpoint().address().to_string(),
      "port", double(rtp->local_endpoint().port())));
}

JSON::Value rtp_create(JSON::Array& v) {
  static int count = 0;

  Resource r;

  boost::intrusive_ptr<Media::Rtp> rtp = new Media::Rtp(); 

  r.close_ = boost::bind(&Media::Rtp::close, rtp);
  r.configure_ = boost::bind(rtp_configure, rtp, _1);

  r.push_source_ = boost::bind(&Media::Rtp::set_sink, rtp, _1);
  r.push_sink_ = Media::make_sink(rtp);
  
  r.pull_sink_ = boost::bind(&Media::Rtp::set_source, rtp, _1);

  std::ostringstream id;

  id << "rtp" << count++;

  g_resource_map[id.str()] = r;

  return JSON::Value(id.str());
}

void fire_filesource_eof(std::string const& id) {
  std::ostream::sentry s(std::cout);

  std::cout 
    << JSON::make_object(
        "method","endOfFile",
        "params", JSON::make_array(id),
        "id", JSON::null)
    << std::endl;
}

JSON::Value filesource_create(JSON::Array& v) {
  static int count = 0;
  Resource r;

  std::ostringstream id;
  id << "filesource" << count++;
 
  boost::intrusive_ptr<Media::FileSource> fsrc = new Media::FileSource(
    boost::get<JSON::String>(boost::get<JSON::String>(v.at(1))).c_str(),
    get_payload_type(boost::get<JSON::String>(v.at(2))),
    wrap_event_callback(boost::bind(fire_filesource_eof, id.str())));

  r.pull_source_ = Media::make_source(fsrc);
  r.close_ = boost::bind(&Media::FileSource::close, fsrc);
  
  g_resource_map[id.str()] = r;
 
  return id.str();
}

JSON::Value filesink_create(JSON::Array& v) {
  static int count = 0;
  Resource r;

  boost::intrusive_ptr<Media::FileSink> fsk = new Media::FileSink(boost::get<JSON::String>(v.at(1)).c_str());

  std::ostringstream id;
  id << "filesink" << count++;

  r.close_ = boost::bind(&Media::FileSink::close, fsk);
  r.push_sink_ = Media::make_sink(fsk);

  g_resource_map[id.str()] = r;

  return id.str();
}

void fire_telephony_event(std::string const& id, int e, boost::posix_time::ptime const& time) {
  std::cout << 
    JSON::make_object(
      "method", "telephony-event",
      "params", JSON::make_array(id, double(e), boost::posix_time::to_simple_string(time)),
      "id", JSON::null) 
    << std::endl;
}

JSON::Value tedetector_create(JSON::Array& v) {
  static int count = 0;
  Resource r;

  std::ostringstream id;
  id << "tedetector" << count++;

  boost::intrusive_ptr<Media::TelephoneEventDetector> ted = new Media::TelephoneEventDetector(wrap_event_callback(boost::bind(fire_telephony_event,id.str(),_1,_2)));

  r.push_sink_ = Media::make_sink(ted);

  g_resource_map[id.str()] = r;

  return id.str();
}

JSON::Value jitterbuffer_create(JSON::Array& v) {
  static int count = 0;
  Resource r;

  std::ostringstream id;
  id << "jitterbuffer" << count++;

  std::pair<Media::Source,Media::Sink> p = make_jitter_buffer(Media::L16);

  r.push_sink_ = p.second;
  r.pull_source_ = p.first;

  g_resource_map[id.str()] = r;

  return id.str();
}

JSON::Value push(JSON::Array& v) {
  ResourceMap::iterator i = g_resource_map.find(boost::get<JSON::String>(v.at(0)));
  if(i != g_resource_map.end() && i->second.push_source_)
    i->second.push_source_(get_sink(v.at(1)));
  else
    throw std::runtime_error("not a push source");

  return JSON::null;
}

JSON::Value pull(JSON::Array& v) { 
  ResourceMap::iterator i = g_resource_map.find(boost::get<JSON::String>(v.at(0)));
  if(i != g_resource_map.end() && i->second.pull_sink_)
    i->second.pull_sink_(get_source(v.at(1)));
  else
    throw std::runtime_error("not a pull sink");
  return JSON::null;
}

JSON::Value destroy(JSON::Array& v) {
  ResourceMap::iterator i = g_resource_map.find(boost::get<JSON::String>(v.at(0)));
  if(i != g_resource_map.end()) {
    i->second.close_();
    g_resource_map.erase(i);
  }
  else
    throw std::runtime_error("resource not found");
  return JSON::null;
}

JSON::Value configure(JSON::Array& v) {
  ResourceMap::iterator i = g_resource_map.find(boost::get<JSON::String>(v.at(0)));
  if(i != g_resource_map.end() && !i->second.configure_.empty())
    return i->second.configure_(v);
  else
    throw std::runtime_error("resource not found");
}

int main(int argc, char* argv[]) {
  try {
    Media::start();
    boost::asio::io_service::work evwork(the_event_io);
    start_event_thread();

    g_methods["create"] = create;
    g_methods["configure"] = configure;
    g_methods["push"] = push;
    g_methods["pull"] = pull;
    g_methods["destroy"] = destroy;

    g_constructors["rtp"] = rtp_create;
    g_constructors["filesink"] = filesink_create;
    g_constructors["filesource"] = filesource_create;
    g_constructors["tedetector"] = tedetector_create;
    g_constructors["jitterbuffer"] = jitterbuffer_create;

    while(std::cin) {
      std::string s;
      std::getline(std::cin, s);

      if(s.length() == 0)
        break;    
    
      JSON::Value v = JSON::parse(s);

      request(boost::get<JSON::Object>(v));
    }
    stop_event_thread();
    //Media::stop();
  }
  catch(std::exception const& e) {
    std::cerr << "Exception occured: " << e.what() << std::endl;
  }
  return 0;
}

