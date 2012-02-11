#ifndef __MEDIA_CORE_HPP__
#define __MEDIA_CORE_HPP__

#include <functional>
#include <memory>
#include <array>
#include <list>
#include <cassert>
#include <cstdint>
#include <boost/asio.hpp>
#include <boost/intrusive/list.hpp>
#include "util.hpp"

namespace Media {

using namespace std::placeholders;

/*
# Basic Concepts

## Source 

is an object that represents asyncronous PDU stream. It must be `MoveConstructible` and implemnent 

  template<typename Handler>
  void operator()(Handler handler)   

`handler` parameter must model `Handler` concept. Calling the operator() initiates operation of
next PDU retrieval. At most one operation can be pending at the time. If source is destroyed while it has
pending handler it will not be invoked. Moving source does not (and MUST not) move pending operation, it has to be
requeued.

`SourceType<Source>::type` must be the type of PDU source is emitting.

## Handler

handler must be callable with PDU type of the source. Ie if F is type of it must implement

  void operator()(F const& f);

Handler must be copy constructible.

## PDU 

Represnets PDU (Protocol Data Unit) of data stream. Can be a RTP packet, audio codec frame etc. Must be
DefaultConstructible, CopyContstuctible, CopyAssignable

## SyncSource


*/

struct Nop {
  typedef void result_type;

  template<typename... Args>
  void operator()(Args... args) {}
};


template<typename T>
struct SharedFunctor {
  template<typename... Args>
  auto operator()(Args&&... args) -> decltype((*((T*)0))(std::forward<Args>(args)...)) const {
    return (*p_)(std::forward<Args>(args)...);  
  }

  template<typename... Args>
  static SharedFunctor<T> make(Args&&... args) {
    SharedFunctor<T> t;
    t.p_ = std::make_shared<T>(std::forward<Args>(args)...);
    return t;
  }

  SharedFunctor(std::shared_ptr<T> const& p) : p_(p) {}

  std::shared_ptr<T> p_; 
};

template<typename A>
SharedFunctor<A> move_to_shared(A&& a) {
  return SharedFunctor<A>::make(std::forward<A>(a));
}

template<typename T>  
struct NoCompose : T { 
  NoCompose(T&& t) : T(t) {}
};

template<typename T>
NoCompose<typename std::decay<T>::type> no_compose(T&& t, typename std::enable_if<std::is_bind_expression<typename std::decay<T>::type>::value>::type* = 0) {
  typedef typename std::decay<T>::type N;
  return NoCompose<N>(std::forward<N&&>(t));
}

template<typename T>
T&& no_compose(T&& t, typename std::enable_if<!std::is_bind_expression<typename std::decay<T>::type>::value>::type* = 0) {
  return t;
}


template<typename T>
struct SourceType {
  typedef typename T::source_type type;
};

template<typename T>
struct SourceType<std::reference_wrapper<T>> {
  typedef typename SourceType<T>::type type;
};

template<typename T>
struct SourceType<const std::reference_wrapper<T>> {
  typedef typename SourceType<T>::type type;
};

template<typename T>
struct SourceType<SharedFunctor<T>> {
  typedef typename SourceType<T>::type type;
};

template<typename T>
struct SourceType<std::function<void (std::function<void (T const&)> const&)>> {
  typedef T type;
};

template<typename T>
struct SourceType<T&> {
  typedef typename SourceType<T>::type type;
};


template<typename A>
struct Pull {
  typedef typename std::remove_reference<decltype((*((A*)(0)))())>::type source_type;

  Pull(A&& a) : a_(std::forward<A>(a)) {}
  Pull(Pull<A>&& p) : a_(std::forward<A>(p.a_)) {}

  static_assert(!std::is_same<source_type, void>::value, "wrong source type");

  template<typename C>
  inline
  void operator()(C const& c) {
    t_ = a_();
    c(t_);
  }

  A a_;
  source_type t_;
};


template<typename A>
Pull<A> pull(A&& a) {
  return Pull<A>(std::forward<A>(a));
}

template<typename A, typename K>
struct Push : K {
  Push(A&& a, K&& k) : K(std::forward<K>(k)), a_(std::forward<A>(a)) {
    start();
  }

  Push(Push&& p) : K(std::forward<K>(p)), a_(std::forward<A>(p.a_)) {
    start();
  }

  Push& operator=(Push&& r) {
    (*static_cast<K*>(this)) = std::move(static_cast<K&&>(r));
    a_ = std::move(r.a_);
    start();
    return *this;
  }

  void start() throw() {
    a_([this](typename SourceType<A>::type const& f) {
      this->operator()(f);
      this->start();
    });
  }

  A a_;
};

template<typename A, typename K>
Push<A,K> push(A&& a, K&& k) {
  return Push<A,K>(std::forward<A>(a),std::forward<K>(k));
}


template<typename F>
struct BranchNode : boost::intrusive::list_base_hook<boost::intrusive::link_mode<boost::intrusive::auto_unlink>> {
  std::function<void (F const& f)> fun_;
};

template<typename A>
struct Root {
  typedef typename SourceType<A>::type source_type;
  typedef boost::intrusive::list<BranchNode<source_type>, boost::intrusive::constant_time_size<false>> Branches;

  Root(A&& a) : a_(std::forward<A>(a)), n_(-1ul), i_(-1ul) {}
  Root(Root&& r) : a_(std::forward<A>(r.a_)), n_(r.n_), i_(r.n_ /*not r._i*/ ) {}
  Root(Root const& r) : a_(std::move(const_cast<Root&>(r).a_)), n_(-1ul), i_(-1ul) {}
  Root() {}
  
  ~Root() {
    for(BranchNode<source_type>& b: branches_) 
      b.fun_ = nullptr;
  }

  template<typename C>
  void operator()(size_t* i, BranchNode<source_type>& b, C c) {
    size_t o = *i;
    *i = i_ + 1;

    if(o == n_) 
      c(f_);
    else {
      b.fun_ = std::move(c);
      branches_.push_back(b);
 
       if(i_ == n_) { // no pending operation
         ++i_;

        a_([this](source_type const& f) mutable {
          f_ = f;
          ++n_;
          assert(n_ == i_);

          Branches brs;
          swap(brs, branches_);
          for(;!brs.empty();) {
            std::function<void (source_type const& f)> t;
            std::swap(t, brs.front().fun_);
            brs.pop_front();
            t(f);
          }
        });
      }
    }
  }
  
  A a_;

  size_t  n_ /*= -1ul*/;
  source_type f_;

  size_t i_ /*= -1ul*/;
  Branches branches_;
};

template<typename A>
Root<A> root(A&& a) {
  return Root<A>(std::forward<A>(a));
}

template<typename Root, typename F>
struct Branch : BranchNode<F> {
  typedef F source_type;
  
  Branch(Root&& r) : root_(std::forward<Root>(r)), i_(0) {}

  template<typename C>
  void operator()(C c) {
    root_(&i_, *this, c);
  }

  Root root_;
  size_t i_ /*= 0*/;
};

template<typename T, typename F = typename SourceType<T>::type>
Branch<T, F> branch(T&& t) {
  return Branch<T,F>(std::forward<T>(t));
}


template<size_t N, typename A, typename F = typename SourceType<A>::type>
struct Assemble {
  Assemble(A&& a) : a_(std::forward<A>(a)) {}

  typedef std::array<F, N> source_type;

  template<typename C>
  void operator()(C c) {
    struct L {
      static void fun(Assemble& a, size_t i, C c, F const& f) {
        a.data_[i] = f;

        if(i == N - 1)
          c(a.data_);
        else
          a.a_(std::bind(fun, std::ref(a), i + 1, no_compose(c), _1));
      }
    };

    a_(std::bind(L::fun, std::ref(*this), 0, no_compose(c), _1));
  }

  A a_;
  std::array<F, N> data_;
};

template<size_t N, typename A>
Assemble<N,A> assemble(A&& a) {
  return Assemble<N,A>(std::forward<A>(a));
}


template<typename A, typename F = typename SourceType<A>::type>
struct Disassemble {
  typedef typename F::value_type source_type;

  Disassemble(A&& a) : a_(std::forward<A>(a)), state_(f_.size()) {}

  template<typename C>
  void operator()(C c) {
    if(state_ == f_.size()) {
      state_ = 1;
      a_([this, c](F const& f) {
        f_ = f;
        c(f_[0]);
      });
    }
    else
      c(f_[state_++]);
  }

  A a_;
  F f_;
  size_t state_ /* = A::source_type::static_size*/;
};

template<typename A>
Disassemble<A> disassemble(A&& a) {
  return Disassemble<A>(std::forward<A>(a));
}


template<typename A, typename T, typename S>
struct Transform {
  typedef T source_type;
  typedef typename SourceType<A>::type F;

  template<typename... Args>
  Transform(A&& a, Args&&... args) : a_(std::forward<A>(a)), s_(std::forward<Args>(args)...) {}

  template<typename C>
  void operator()(C c) {
    static_assert(!std::is_reference<S>::value, "S must not be a reference");
   
    a_([this, c](F const& f) {
        T t;
        s_ = transform(f, t, std::move(s_));
        c(t); 
      });
  }

  A a_;
  S s_;
};

template<typename A, typename T>
struct Transform<A, T, void> {
  typedef T source_type;
  typedef typename SourceType<A>::type F;

  Transform(A&& a) : a_(std::forward<A>(a)) {}
  Transform(Transform&& t) : a_(std::move(t.a_)) {}

  template<typename C>
  void operator()(C c) {
    a_([c](F const& f) {
      T t;
      transform(f, t);
      c(t);
    });
  }

  A a_;
};


template<typename A, typename T>
struct Filter {
  typedef T source_type;

  Filter(A&& a) : a_(std::forward<A>(a)) {}

  //template<typename C>
  void operator()(std::function<void (T const& t)> const& c) {
    Filter<A, T>* self = this;
    a_([=](std::pair<bool, T> const& t) {
      if(t.first)
        c(t.second);
      else
        (*self)(c);
    });
  }

  A a_;
};

template<typename T, typename A, typename... Args, typename F = typename SourceType<A>::type>
auto transform_to(A&& a, Args&&... args) -> Transform<A, T, typename std::decay<decltype(transform(typename SourceType<A>::type(), *((T*)0)))>::type> {
  return Transform<A, T, typename std::decay<decltype(transform(typename SourceType<A>::type(), *((T*)0)))>::type>(std::forward<A>(a), std::forward<Args>(args)...);
}

template<typename T, typename A, typename... Args, typename F = typename SourceType<A>::type>
auto transform_to(A&& a, Args&&... args) -> Filter<Transform<A, std::pair<bool, T>, typename std::decay<decltype(transform(F(), *((std::pair<bool,T>*)0)))>::type>, T> {
  return Transform<A, std::pair<bool, T>, typename std::decay<decltype(transform(F(), *((std::pair<bool,T>*)0)))>::type>(std::forward<A>(a), std::forward<Args>(args)...);
}


template<typename A, typename D>
struct Condition {
  Condition(A&& a, D&& d) : a_(std::forward<A>(a)), d_(std::forward<D>(d)) {
    a_(std::bind(&Condition<A,D>::pull, this, _1));
  }

  Condition(Condition&& r) : a_(std::move(r.a_)), d_(std::move(r.d_)) {
    a_(std::bind(&Condition<A,D>::pull, this, _1));
  }


  Condition& operator=(Condition&& r) {
    a_ = std::move(r.a_);
    d_ = std::move(r.d_);
    a_(std::bind(&Condition<A,D>::pull, this, _1));
    return *this;
  }

  void pull(typename SourceType<A>::type const&) {
    d_();
    //a_(std::bind(&Condition<A,D>::pull, this, _1))
  }

  A a_;
  D d_;
};

template<typename A, typename D>
Condition<A, D> condition(A&& a, D&& d) {
  return Condition<A,D>(std::forward<A>(a), std::forward<D>(d));
}


template<typename P>
struct DelayedSource {
  typedef P source_type;

  DelayedSource() {}
  DelayedSource(DelayedSource&& d) : w_(std::move(d.w_)) {}

  template<typename A>
  DelayedSource(A&& a) : w_(wrap(std::forward<A>(a))) {}

  template<typename A>
  DelayedSource& operator=(A&& a) {
    w_ = wrap(std::forward<A>(a));
    std::function<void (P const&)> c;
    std::swap(c_, c);
    if(c) (*w_)(c);
    return *this;
  }

  DelayedSource& operator=(DelayedSource const&) = delete;

  void operator()(std::function<void (P const&)> c) {
    if(w_)
      (*w_)(std::move(c));
    else
      c_ = std::move(c);
  }

  template<typename T>
  void operator()(SharedFunctor<T> const& c) {
    (*this)(std::function<void (P const&)>(c));
  }

  struct Wrapper {
    virtual ~Wrapper() {}
    virtual void operator()(std::function<void (P const&)>) =0;
  };

  template<typename A>
  std::unique_ptr<Wrapper> wrap(A&& a) {
    struct W : Wrapper {
      W(A && a) : a_(std::forward<A>(a)) {}
      void operator()(std::function<void (P const& p)> c) {
        a_(std::move(c));
      }
      A a_;
    };

    return std::unique_ptr<Wrapper>(new W(std::forward<A>(a)));
  }

  std::unique_ptr<Wrapper> w_;
  std::function<void (P const&)> c_;
};

extern boost::asio::io_service g_io;

void start();
void stop();

}

#endif
