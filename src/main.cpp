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

struct Resource {
  Media::PullSink pull_sink_;
  Media::Source pull_source_;
  
  Media::Sink   push_sink_;
  Media::PushSource push_source_;

  boost::function<void()> close_;

  boost::function<void (JSON::Value&)> configure_;
};

typedef std::map<std::string, Resource> ResourceMap;
ResourceMap the_resource_map;

Media::Source get_source(JSON::Value& v) {
  if(boost::get<std::string>(&v)) {
    ResourceMap::iterator i = the_resource_map.find(boost::get<std::string>(v));

    if(i == the_resource_map.end() || i->second.pull_source_.empty())
      throw std::runtime_error("not a pull source");

    return i->second.pull_source_;
  }
  else if(boost::get<JSON::Object>(&v)) {
    JSON::Object& tr = boost::get<JSON::Object>(v);
    std::string const& type = boost::get<std::string>(tr["type"]);
    if(type == "decoder"){
      std::string const& codec = boost::get<std::string>(tr["codec"]);
      if(codec == "g771a") {
        return Media::decoder<Media::G711aDecoder>(get_source(tr["source"]));
      }
      else if(codec == "g711u") {
        return Media::decoder<Media::G711uDecoder>(get_source(tr["source"]));
      }
      else {
        throw std::runtime_error("unsupported codec");
      }
    }
    else if(type == "encoder") {
      std::string const& codec = boost::get<std::string>(tr["codec"]);
      if(codec == "g771a") {
        return Media::encoder<Media::G711aEncoder>(get_source(tr["source"]));
      }
      else if(codec == "g711u") {
        return Media::encoder<Media::G711uEncoder>(get_source(tr["source"]));
      }
      else {
        throw std::runtime_error("unsupported codec");
      }
    }
    else if(type == "mixer") {
      return get_source(tr["sources"]);

    }
    else {
      throw std::runtime_error("unsupported transformation");
    }
  }
  else if(boost::get<JSON::Array>(&v)) {
    JSON::Array a;
    if(a.size() == 0)
      throw std::runtime_error("mixer can't have empty source list");

    Media::Source s = get_source(a[0]);

    for(int i = 1; i != a.size(); ++i)
      s = Media::mix2(s, get_source(a[i]));

    return s;
  }
  else
    throw std::runtime_error("source can be source id or transformation object");
}

Media::Sink get_sink(JSON::Value& v) {
  if(boost::get<std::string>(&v)) {
    ResourceMap::iterator i = the_resource_map.find(boost::get<std::string>(v));

    if(i == the_resource_map.end() || i->second.push_sink_.empty())
      throw std::runtime_error("not a push sink");

    return i->second.push_sink_;
  }
  else if(boost::get<JSON::Object>(&v)) {
    JSON::Object& tr = boost::get<JSON::Object>(v);
    std::string const& type = boost::get<std::string>(tr["type"]);
    if(type == "decoder"){
      std::string const& codec = boost::get<std::string>(tr["codec"]);
      if(codec == "g771a") {
        return Media::decoder<Media::G711aDecoder>(get_sink(tr["sink"]));
      }
      else if(codec == "g711u") {
        return Media::decoder<Media::G711uDecoder>(get_sink(tr["sink"]));
      }
      else {
        throw std::runtime_error("unsupported codec");
      }
    }
    else if(type == "encoder") {
      std::string const& codec = boost::get<std::string>(tr["codec"]);
      if(codec == "g771a") {
        return Media::encoder<Media::G711aEncoder>(get_sink(tr["sink"]));
      }
      else if(codec == "g711u") {
        return Media::encoder<Media::G711uEncoder>(get_sink(tr["sink"]));
      }
      else {
        throw std::runtime_error("unsupported codec");
      }
    }
    else if(type == "splitter") {
      return get_sink(tr["sinks"]);
    }
    else {
      throw std::runtime_error("unsupported transformation");
    }
  }
  else if(boost::get<JSON::Array>(&v)) {
    JSON::Array a;
    if(a.size() == 0)
      throw std::runtime_error("splitter can't have empty sink list");

    Media::Sink s = get_sink(a[0]);

    for(int i = 1; i != a.size(); ++i)
      s = Media::split2(s, get_sink(a[i]));

    return s;
  }
  else {
    throw std::runtime_error("sink can be sink id or transformation object");
  }
}

void push(JSON::Value& v) {
  ResourceMap::iterator i = the_resource_map.find(boost::get<std::string>(boost::get<JSON::Object>(v)["source"]));
  if(i != the_resource_map.end() && i->second.push_source_) {
    i->second.push_source_(get_sink(boost::get<JSON::Object>(v)["sink"]));
  }
  else throw std::runtime_error("not a push source");
}

void pull(JSON::Value& v) { 
  ResourceMap::iterator i = the_resource_map.find(boost::get<std::string>(boost::get<JSON::Object>(v)["sink"]));
  if(i != the_resource_map.end() && i->second.pull_sink_) {
    i->second.pull_sink_(get_source(boost::get<JSON::Object>(v)["source"]));
  }
  else throw std::runtime_error("not a pull sink");
}

void close(JSON::Value& v) {
  ResourceMap::iterator i = the_resource_map.find(boost::get<std::string>(v));
  if(i != the_resource_map.end()) {
    i->second.close_();
    the_resource_map.erase(i);
  }
  else throw std::runtime_error("resource not found");
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

void configure_rtp(boost::intrusive_ptr<Media::Rtp> const& rtp, JSON::Value& v) {
  JSON::Object& a = boost::get<JSON::Object>(v);

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
    rtp->connect(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(boost::get<std::string>(ep["address"])), boost::get<double>(ep["port"])));
  }

  i = a.find("local");
  if(i != a.end()) {
    JSON::Object& ep = boost::get<JSON::Object>(i->second);
    rtp->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(boost::get<std::string>(ep["address"])), boost::get<double>(ep["port"])));
  }
}

static int rtp_count = 0; 

std::string create_rtp(JSON::Value& v) {
  Resource r;

  boost::intrusive_ptr<Media::Rtp> rtp = new Media::Rtp(); 

  r.close_ = boost::bind(&Media::Rtp::close, rtp);
  r.configure_ = boost::bind(configure_rtp, rtp, _1);

  r.push_source_ = boost::bind(&Media::Rtp::set_sink, rtp, _1);
  r.push_sink_ = Media::make_sink(rtp);
  
  r.pull_sink_ = boost::bind(&Media::Rtp::set_source, rtp, _1);

  r.configure_(v);

  std::ostringstream id;

  id << "rtp" << rtp_count++;

  the_resource_map[id.str()] = r;

  std::ostringstream json;

  json << "{\"status\":\"ok\",\"rtp\":\"" << id.str() 
    << "\",\"address\":{\"address\":\"" << rtp->local_endpoint().address() 
    <<  "\",\"port\":" << rtp->local_endpoint().port() << "}}";

  return json.str();
}

Media::PayloadType get_payload_type(std::string const& name) {
  for(int i = 0; i < sizeof(Media::Payloads)/sizeof(*Media::Payloads); ++i)
    if(name == Media::Payloads[i]->name())
      return Media::Payloads[i];

  throw std::runtime_error("unknown codec");
}

static int filesource_count = 0;

void fire_filesource_eof(std::string const& id) {
  std::ostream::sentry sentry(std::cout);
  std::cout << "{\"event\":\"eof\",\"file\":\"" << id << "\"}" << std::endl;
}

std::string create_filesource(JSON::Value& v) {
  Resource r;

  JSON::Object& options = boost::get<JSON::Object>(v);

  std::ostringstream id;
  id << "filesource" << filesource_count++;
 
  boost::intrusive_ptr<Media::FileSource> fsrc = new Media::FileSource(
    boost::get<std::string>(options["path"]).c_str(),
    get_payload_type(boost::get<std::string>(options["encoding"])),
    wrap_event_callback(boost::bind(fire_filesource_eof, id.str())));

  r.pull_source_ = Media::make_source(fsrc);
  r.close_ = boost::bind(&Media::FileSource::close, fsrc);
  
  the_resource_map[id.str()] = r;
 
  return "{\"status\":\"ok\", \"id\":\""+id.str() + "\"}";
}

static int filesink_count = 0;

std::string create_filesink(JSON::Value& v) {
  Resource r;

  JSON::Object& options = boost::get<JSON::Object>(v);

  boost::intrusive_ptr<Media::FileSink> fsk = new Media::FileSink(boost::get<std::string>(options["path"]).c_str());

  std::ostringstream id;
  id << "filesink" << filesink_count++;

  r.close_ = boost::bind(&Media::FileSink::close, fsk);
  r.push_sink_ = Media::make_sink(fsk);

  the_resource_map[id.str()] = r;

  return "{\"status\":\"ok\",\"id\":\"" + id.str() + "\"}";
}

void fire_telephony_event(std::string const& id, int e, boost::posix_time::ptime const&) {
  std::ostream::sentry sentry(std::cout);

  std::cout << "{\"event\":\"telephony-event\",\"detector\":\"" << id << "\",\"value\":" << e << "}" << std::endl; 
}

static int tedetector_count = 0;
std::string create_tedetector(JSON::Value& v) {
  Resource r;
  JSON::Object& options = boost::get<JSON::Object>(v);

  std::ostringstream id;
  id << "tedetector" << tedetector_count++;
  boost::intrusive_ptr<Media::TelephoneEventDetector> ted = new Media::TelephoneEventDetector(wrap_event_callback(boost::bind(fire_telephony_event,id.str(),_1,_2)));

  r.push_sink_ = Media::make_sink(ted);

  the_resource_map[id.str()] = r;

  return "{\"status\":\"ok\",\"id\":\"" + id.str() + "\"}";
}

static int jb_count = 0;
std::string create_jitter_buffer(JSON::Value& v) {
  Resource r;
  JSON::Object& options = boost::get<JSON::Object>(v);

  std::ostringstream id;
  id << "jitterbuffer" << jb_count++;

  JSON::Value length = options["latency"];

  std::pair<Media::Source,Media::Sink> p = make_jitter_buffer(
    get_payload_type(boost::get<std::string>(options["codec"])),
    boost::posix_time::milliseconds(length.empty() ? 50 : boost::get<double>(length)));

  r.push_sink_ = p.second;
  r.pull_source_ = p.first;

  the_resource_map[id.str()] = r;
  return "{\"status\":\"ok\",\"id\":" + id.str() + "\"}";
}

std::string create(JSON::Value& v) {
  JSON::Object& o = boost::get<JSON::Object>(v);
  std::string const& type = boost::get<std::string>(o["type"]);

  if(type == "rtp") {
    return create_rtp(v);  
  }
  else if(type == "filesource") {
    return create_filesource(v);
  }
  else if(type == "filesink") {
    return create_filesink(v);
  }
  else if(type == "tedetector") {
    return create_tedetector(v);
  }
  else if(type == "jitterbuffer") {
    return create_jitter_buffer(v);
  }

  throw std::runtime_error("unknown resource type");
}

std::string request(JSON::Value& v) {
  try {
    std::string const& rq = boost::get<std::string>(boost::get<JSON::Object>(v)["request"]);

    if(rq == "create") {
      return create(v);
    }
    else if(rq == "pull") {
      pull(v);
    }
    else if(rq == "push") {
      push(v);
    }
    else if(rq == "close") {
      close(v);
    }
    else if(rq == "configure") {
      ResourceMap::iterator i = the_resource_map.find(boost::get<std::string>(boost::get<JSON::Object>(v)["target"]));

      if(i == the_resource_map.end() || i->second.configure_.empty())
        throw std::runtime_error("not a configurabel resource");

      i->second.configure_(v);
    } 
    else {
      return "{\"status\":\"failure\", \"message\":\"unknown request\"}"; 
    }

    return "{\"status\":\"ok\"}";
  }
  catch(std::exception const& e) {
    return std::string("{\"status\":\"failure\",\"message:\"") + e.what() + "\"}";
  }
}

int main(int argc, char* argv[]) {
  try {
    Media::start();
    boost::asio::io_service::work evwork(the_event_io);
    start_event_thread();
    while(std::cin) {
      std::string s;
      std::getline(std::cin, s);

      if(s.length() == 0)
        break;    
    
      JSON::Value v = JSON::parse(s);

      std::cout << request(v) << std::endl;
    }
    stop_event_thread();
    //Media::stop();
  }
  catch(std::exception const& e) {
    std::cerr << "Exception occured: " << e.what() << std::endl;
  }
  return 0;
}
