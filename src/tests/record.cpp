#include "../media.hpp"
#include <functional>

template<size_t... Indices>
struct IndexList {
  typedef IndexList<Indices..., sizeof...(Indices)> succ;
};

template<size_t I>
struct MakeIndexList {
  typedef typename MakeIndexList<I-1>::type::succ type;
};

template<>
struct MakeIndexList<0> {
  typedef IndexList<> type;
};

template<size_t... I1, size_t... I2>
IndexList<I1...,I2...> concat(IndexList<I1...>, IndexList<I2...>) { return IndexList<I1...,I2...>(); }

template<size_t N, size_t... I>
IndexList<I+N ...> shift(IndexList<I...>) { return IndexList<I+N ...>(); }

template<typename F, typename... Types, size_t... I>
auto map(std::tuple<Types...>& t, F f, IndexList<I...>) -> decltype(std::make_tuple(f(std::get<I>(t))...)) {
  static_assert(sizeof...(Types) == sizeof...(I), "Types and Indices should have same dimensions");
  return std::make_tuple(f(std::get<I>(t))...);
}

template<typename F, typename... Types>
auto map(std::tuple<Types...>& t, F f) -> decltype(map(t, f, *(typename MakeIndexList<sizeof... (Types)>::type*)nullptr)) {
  typename MakeIndexList<sizeof...(Types)>::type i;
  return map(t, f, i);
}

template<typename F, typename... Types, typename... Types2, size_t... I>
auto map(std::tuple<Types...>& t, std::tuple<Types2...>& t2, F f, IndexList<I...>) -> decltype(std::make_tuple(f(std::get<I>(t), std::get<I>(t2))...)) {
  return std::make_tuple(f(std::get<I>(t), std::get<I>(t2))...);
}

template<typename F, typename... Types, typename... Types2>
auto map(std::tuple<Types...>& t, std::tuple<Types2...>& t2, F f) -> decltype(map(t, t2, f, *(typename MakeIndexList<sizeof... (Types)>::type*)nullptr)) {
  return map(t, t2, f, typename MakeIndexList<sizeof... (Types)>::type());
}

////////////////////////

using namespace Media;
using namespace Media::Audio;
using namespace Media::Transport;

template<typename... Sources>
struct MultiDecoder {
  typedef LinearFrame<80> source_type;

  MultiDecoder(Sources&&... source) : sources_(root(std::forward<Sources>(source))...) {
    activate<0>();
  }

  MultiDecoder(MultiDecoder&& r) : sources_(std::move(r.sources_)) {
    activate<0>();
  }

  template<typename Base, typename... Conditions>
  struct State : Base {
    State(Base&& base, Conditions&&... conditions) : Base(std::forward<Base>(base)), conditions_(std::forward<Conditions>(conditions)...) {}
  
    std::tuple<Conditions...> conditions_;
  };

  template<typename Base, typename... Conditions>
  State<Base, Conditions...> make_state(Base&& base, Conditions&&... conditions) {
    return State<Base, Conditions...>(std::forward<Base>(base), std::forward<Conditions>(conditions)...);
  }

  template<size_t Active, size_t... Conditions>
  void activate_helper(IndexList<Conditions...>)
  {
    auto a = make_state(
      resize<80>(decode(jitter(branch(std::get<Active>(sources_))))),
      std::move(condition(branch(std::get<Conditions>(sources_)), std::bind(&MultiDecoder::activate<Conditions>, this)))...);
    auto p = std::make_shared<decltype(a)>(std::move(a));
    fun_ = [p](std::function<void (LinearFrame<80> const&)> const& f) { (*p)(f); };  
  }

  template<size_t I>
  void activate() {
    activate_helper<I>(concat(typename MakeIndexList<I>::type(), shift<I+1>(typename MakeIndexList<sizeof...(Sources)-(I+1)>::type())));
  }

  template<typename C>
  void operator()(C c) {
    fun_(c);
  }

  std::function<void (std::function<void (Media::Audio::LinearFrame<80> const&)> const&)> fun_;

  std::tuple<Media::Root<Sources>...> sources_;
};

template<typename... Sources>
MultiDecoder<Sources...> multi_decode(Sources&&... sources) {
  return MultiDecoder<Sources...>(std::forward<Sources>(sources)...);
}

void setup() {
  using namespace Media;
  using namespace Media::Transport;
  using namespace Media::Audio;

  Socket s;

  s.s_.open(boost::asio::ip::udp::v4());
  s.s_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 8000));

  std::shared_ptr<void> w;

  auto rtp = root(transform_to<RtpPacket>(std::move(s)));
  
  g_io.post([&]() {
    auto a = push(
      rate(multi_decode(
        transform_to<Packet<UlawFrame<160>>>(branch(std::ref(rtp)), 0),
        transform_to<Packet<UlawFrame<240>>>(branch(std::ref(rtp)), 0),
        transform_to<Packet<AlawFrame<160>>>(branch(std::ref(rtp)), 8),
        transform_to<Packet<AlawFrame<240>>>(branch(std::ref(rtp)), 8),
        transform_to<Packet<IlbcFrame<160>>>(branch(std::ref(rtp)), 100),
        transform_to<Packet<IlbcFrame<240>>>(branch(std::ref(rtp)), 100))),
      Writer(STDOUT_FILENO));
    
    w = std::make_shared<decltype(a)>(std::move(a));
  });

  pause();
}

int main(int argc, char* agv[]) {
  Media::start();
   
  setup();

  Media::stop();
}

