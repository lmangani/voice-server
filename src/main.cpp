#include <stdio.h>
#include "media.hpp"
#include "json.hpp"
#include <map>
#include <set>
#include <list>
#include <string>
#include <boost/bind.hpp>

struct Resource {
  void update_sinks();
  void update_sources();

  boost::intrusive_ptr<Media::Adaptor> source_;
  std::set<std::string> sinks_;  

  Media::Sink sink_;
  Media::Sink mixer_;
  
  std::set<std::string> sources_;
  
  Media::Formats acceptable_formats_;

  boost::function<JSON::Value (Resource&, JSON::Array&)> configure_;
};

typedef std::map<std::string, Resource> Resources;
Resources g_resources;

void fire_event(std::string const& resource, std::string const& event) {
  
}


Resource& get_resource(std::string const& id) {
  Resources::iterator i = g_resources.find(id);

  if(i == g_resources.end()) throw std::runtime_error("Invalid Resource Id");

  return i->second;
}

void Resource::update_sinks() {
  std::list<std::pair<Media::Sink, Media::Formats> > sinks;
  for(std::set<std::string>::iterator i = sinks_.begin(), e = sinks_.end(); i!=e; ++i) {
    Resource& s = get_resource(*i);
    if(s.sources_.size() > 1)
      sinks.push_back(std::make_pair(s.mixer_, 
        Media::Formats(Media::linear, s.acceptable_formats_.ptime_ == Media::packetAny ? Media::packet20ms : s.acceptable_formats_.ptime_, true)));
    else
      sinks.push_back(std::make_pair(s.sink_, s.acceptable_formats_));
  }

  set_sinks(source_, sinks);
}

void Resource::update_sources() {
  if(sources_.size() > 1 && !mixer_) {
    mixer_ = Media::mixer(sink_);
  }
  else
    mixer_ = Media::Sink();
}

void connect(std::string const& source_id, std::string const& sink_id) {
  Resource& sink = get_resource(sink_id);
  Resource& source = get_resource(source_id);

  source.sinks_.insert(sink_id);
  sink.sources_.insert(source_id);

  sink.update_sources();
  source.update_sinks();
}

void disconnect(std::string const& source_id, std::string const& sink_id) {
  Resource& sink = get_resource(sink_id);
  Resource& source = get_resource(source_id);

  source.sinks_.erase(sink_id);
  sink.sources_.erase(sink_id);

  sink.update_sources();
  source.update_sinks();
}

struct RtpMap {
  int operator()(Media::Encoding e) const {
    return map_[e];
  }

  Media::Encoding operator()(int e) const {
    const int* i = std::find(map_, map_ + Media::last_encoding, e);
    if(i != map_ + Media::last_encoding) return Media::Encoding(i - map_);
    return Media::linear;  
  }

  int map_[Media::last_encoding];
};

JSON::Value rtp_configure(boost::intrusive_ptr<Media::Rtp> const& rtp, Resource& resource, JSON::Array& args) {
  JSON::Object& params = boost::get<JSON::Object>(args.at(1));

  if(!(params["rtpmap"] == JSON::null)) {
    JSON::Object& m = boost::get<JSON::Object>(params["rtpmap"]);
    
    RtpMap rtpmap = {{0}};
    
    if(!(m["pcma"] == JSON::null))
      rtpmap.map_[Media::pcma] = static_cast<int>(boost::get<double>(m["pcma"]));
    
    if(!(m["pcmu"] == JSON::null))
      rtpmap.map_[Media::pcmu] = static_cast<int>(boost::get<double>(m["pcmu"]));
    
    if(!(m["ilbc"] == JSON::null))
      rtpmap.map_[Media::ilbc] = static_cast<int>(boost::get<double>(m["ilbc"]));
    
    set_rtp_map(rtp, boost::function<int (Media::Encoding)>(boost::bind<int>(rtpmap, _1)));
    set_rtp_map(rtp, boost::function<Media::Encoding (int)>(boost::bind<Media::Encoding>(rtpmap, _1)));

    Media::Formats formats;
    for(int i = Media::first_encoding, e = Media::last_encoding; i != e; ++i)
      formats.mask_ |= (1 << i);

    resource.acceptable_formats_ = formats;
    resource.update_sources();
    resource.update_sinks();   
  }

  if(params["local"] != JSON::null) {
    JSON::Object& address = boost::get<JSON::Object>(params["local"]);
    boost::asio::ip::udp::endpoint ep;
    
    ep.port(boost::get<double>(address["port"]));
    if(address["address"] != JSON::null)
      ep.address(boost::asio::ip::address::from_string(boost::get<JSON::String>(address["address"])));

    set_local_endpoint(rtp, ep);
  }

  if(params["remote"] != JSON::null) {
    JSON::Object& address = boost::get<JSON::Object>(params["remote"]);
    boost::asio::ip::udp::endpoint ep;
    
    ep.port(boost::get<double>(address["port"]));
    ep.address(boost::asio::ip::address::from_string(boost::get<JSON::String>(address["address"])));

    set_remote_endpoint(rtp, ep);
  }

  return JSON::make_object(
    "local", JSON::make_object(
      "address", get_local_endpoint(rtp).address().to_string(),
      "port", double(get_local_endpoint(rtp).port())));
}

JSON::Value rtp_create(JSON::Array& args) {
  static int count = 0;

  Resource r;

  boost::intrusive_ptr<Media::Rtp> rtp = Media::make_rtp();

  r.source_ = Media::make_adaptor();
  get_source(rtp)(get_sink(r.source_));

  r.sink_ = get_sink(rtp);

  r.configure_ = boost::bind(rtp_configure, rtp, _1, _2);

  std::ostringstream id;

  id << "rtp" << count++;

  g_resources[id.str()] = r;

  return JSON::Value(id.str());
} 


JSON::Value filesource_create(JSON::Array& args) {
  static int count = 0;

  std::ostringstream id;

  id << "filesource" << count++;

  JSON::String const& enc = boost::get<JSON::String>(args.at(2));

  Media::Format f;
  f.ptime_ = Media::packet20ms;
  f.dejittered_ = true;

  if(enc == "pcma")
    f.encoding_ = Media::pcma;
  else if(enc == "pcmu")
    f.encoding_ = Media::pcmu;
  else if(enc == "ilbc")
    f.encoding_ = Media::ilbc;
  else if(enc == "linear")
    f.encoding_ = Media::linear;
  else
    throw std::runtime_error("Unknown Encoding");

  boost::intrusive_ptr<Media::FileSource> fs = Media::make_file_source(
    boost::get<JSON::String>(args.at(1)).c_str(), f, boost::bind(fire_event, id.str(), "eof"));

  Resource r;
  r.source_ = Media::make_adaptor();
  get_source(fs)(get_sink(r.source_));

  g_resources[id.str()] = r;

  return id.str();
}


void start_event_thread() {}
void stop_event_thread() {}

typedef std::map<std::string, boost::function<JSON::Value (JSON::Array& )> > MethodsMap;
MethodsMap g_methods;

MethodsMap g_constructors;

JSON::Value create(JSON::Array& params) {
  MethodsMap::iterator i = g_constructors.find(boost::get<JSON::String>(params.at(0)));

  if(i == g_constructors.end())
    throw std::runtime_error("Unknown Constructor");

  return i->second(params);  
}

JSON::Value close(JSON::Array& params) {
  Resources::iterator i = g_resources.find(boost::get<JSON::String>(params.at(0)));

  if(i != g_resources.end()) {
    std::for_each(i->second.sources_.begin(), i->second.sources_.end(),
      boost::bind((void (*)(std::string const&, std::string const&))disconnect, _1, i->first));
    std::for_each(i->second.sinks_.begin(), i->second.sinks_.end(),
      boost::bind((void (*)(std::string const&, std::string const&))disconnect, i->first, _1));

    g_resources.erase(i);
  }

  return JSON::null; 
}

JSON::Value configure(JSON::Array& params) {
  Resource& r = get_resource(boost::get<JSON::String>(params.at(0)));

  if(r.configure_)
    return r.configure_(r, params);

  return JSON::null;
}

JSON::Value connect(JSON::Array& params) {
  connect(boost::get<JSON::String>(params.at(0)), boost::get<JSON::String>(params.at(1)));  
  return JSON::null;
}

JSON::Value disconnect(JSON::Array& params) {
  disconnect(boost::get<JSON::String>(params.at(0)), boost::get<JSON::String>(params.at(1)));
  return JSON::null;
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


int main(int argc, char* argv[]) {
  try {
    Media::start();
    start_event_thread();
    std::cout << std::boolalpha;

    g_methods["create"] = create;
    g_methods["configure"] = configure;
    g_methods["connect"] = (JSON::Value (*)(JSON::Array&))connect;
    g_methods["disconnect"] = (JSON::Value (*)(JSON::Array&))disconnect;

    g_constructors["rtp"] = rtp_create;
    g_constructors["filesource"] = filesource_create;
    
    while(std::cin) {
      JSON::Value v = JSON::parse(std::cin);
      if(!(v == JSON::null))
        request(boost::get<JSON::Object>(v));
    }
    stop_event_thread();
    Media::stop();
  }
  catch(std::exception const& e) {
    std::cerr << "Exception occured: " << e.what() << std::endl;
  }
  return 0;
}

