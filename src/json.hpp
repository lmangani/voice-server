#include <string>
#include <map>
#include <vector>
#include <boost/variant.hpp>
#include <boost/noncopyable.hpp>

namespace JSON {
struct NullType {};

inline
bool operator != (NullType const&, NullType const&) { return false; }

inline
bool operator == (NullType, NullType) { return true; }

inline
std::ostream& operator << (std::ostream& os, NullType) { return os << "null"; }

struct String : std::string {
  String() {}
  String(std::string const& s) : std::string(s) {}
};

inline
std::ostream& operator << (std::ostream& os, String const& s) {
  return os << "\"" << static_cast<std::string const&>(s) << "\"";
}

typedef boost::make_recursive_variant<
  NullType,
  double,
  String,
  bool,
  std::map<std::string, boost::recursive_variant_>,
  std::vector<boost::recursive_variant_>
>::type Value;

static const Value null = NullType();

typedef std::map<std::string, Value> Object;
typedef std::vector<Value> Array;

inline
bool operator != (Value const& a, Value const& b) {
  return !(a == b);
}

inline
std::ostream& operator << (std::ostream& os, JSON::Object const& m) {
  os << "{";

  for(JSON::Object::const_iterator i = m.begin(), e = m.end();i!=e;) {
    os << "\"" << i->first << "\":" << i->second;
    if((++i) == e)
      break;
    os << ",";
  }

  return os << "}";
}

inline
std::ostream& operator << (std::ostream& os, JSON::Array const& m) {
  os << "[";

  for(JSON::Array::const_iterator i = m.begin(), e = m.end();i!=e;) {
    os << *i;
    if((++i) == e)
      break;
    os << ",";
  }

  return os << "]";
}

inline
Object make_object(std::string const& n, JSON::Value const& v) {
  Object o;
  o[n] = v;
  return o;
}

inline
Object make_object(std::string const& n, Value const& v,
  std::string const& n1, Value const& v1) {
  Object o;
  o[n] = v;
  o[n1] = v1;
  return o;
}

inline
Object make_object(std::string const& n, Value const& v,
  std::string const& n1, Value const& v1,
  std::string const& n2, Value const& v2) {
  Object o;
  o[n] = v;
  o[n1] = v1;
  o[n2] = v2;
  return o;
}

inline
Array make_array(JSON::Value const& v) {
  Array a;
  a.push_back(v);
  return a;
}

inline
Array make_array(JSON::Value const& v, JSON::Value const& v1) {
  Array a;
  a.push_back(v);
  a.push_back(v1);
  return a;
}

inline
Array make_array(JSON::Value const& v, JSON::Value const& v1, JSON::Value const& v2) {
  Array a;
  a.push_back(v);
  a.push_back(v1);
  a.push_back(v2);
  return a;
}

struct Error : std::exception {
  const char* what() const throw() { return "JSON parser error"; } 
};

namespace Detail {
struct Reader : boost::noncopyable {
  Reader(std::istream& is) : is_(is), c_(std::istream::traits_type::eof()) {}
  
  Reader& operator ++ () {
    c_ = std::istream::traits_type::eof(); 
    return *this;
  }

  int operator * () {
    if(c_ == std::istream::traits_type::eof() && is_)
      c_ = is_.get();
    return c_;
  }

  typedef void (Reader::*Dummy)();

  operator Dummy() { return (operator * ()) != std::istream::traits_type::eof() ? &Reader::dummy : 0; };

  void dummy() {} 
  
  std::istream& is_;
  int c_;
};

Value parse(Reader&);
std::string parse_string(Reader&);

inline
void skipws(Reader& b) {
  for(;b && isspace(*b); ++b);
}

inline
char token(Reader& b) {
  skipws(b);
  if(!b) throw Error();
  return *b;
}

inline
char next(Reader& b) {
  return token(++b);
}

inline
void assert_symbol(Reader& b, char c) {
  if(token(b) != c) throw Error();
}

Value parse_object(Reader& b) {
  std::map<std::string, Value> o;
  
  assert_symbol(b, '{');
  if(next(b) == '}') {
    ++b;
    return o;
  }
   
  for(;;) {
    std::string name = parse_string(b);
    
    if(next(b) != ':') throw Error();
    
    next(b);

    o[name] = parse(b);

    skipws(b);
    if(!b) throw Error();

    if(*b == '}') {
      ++b;
      return o;
    }
    if(*b != ',')
      throw Error();

    next(b);
  }
}

Value parse_array(Reader& b) {
  std::vector<Value> v;

  assert_symbol(b, '[');
  if(next(b) == ']') {
    ++b;
    return v;
  }

  for(;;) {
    v.push_back(parse(b));
    skipws(b);
    if(!b) throw Error();

    if(*b == ']') {
      ++b;
      return v;
    }
    
    if(*b != ',') throw Error();

    next(b);    
  }
}

void convert_ucs2_to_utf8(int ucs2, std::string& s) {
  if(ucs2 < 0x80) {
    s.push_back(ucs2);
  }
  else if(ucs2 >= 0x80  && ucs2 < 0x800) {
    s.push_back((ucs2 >> 6)|0xC0);
    s.push_back((ucs2 & 0x3F)|0x80);
  }
  else  if(ucs2 >= 0x800 && ucs2 < 0xFFFF) {
    s.push_back((ucs2 >> 12) | 0xE0);
    s.push_back(((ucs2 >> 6) & 0x3F) | 0x80);
    s.push_back((ucs2 & 0x3F) | 0x80);
  }
}

std::string parse_string(Reader& b) {
  std::string s;
  
  assert_symbol(b, '"');

  for(++b;;++b) {
    switch(*b) {
    case '"':
      ++b;
      return s;
    case '\\':
      ++b;
      if(!b) throw Error();
      switch(*b) {
      case 'b':
        s.push_back('\b');
        break;
      case 'f':
        s.push_back('\f');
        break;
      case 'n':
        s.push_back('\n');
      case 'r':
        s.push_back('\r');
      case 't':
        s.push_back('\t');
      case 'u': {
          ++b;
          unsigned u = 0;
          for(int i = 0; i < 4; ++i, ++b) {
            if(!b) throw Error();
            u *= 16;
            if(*b >= '0' && *b <= '9')
              u += *b - '0';
            else if(*b >= 'a' && *b <= 'f')
              u += *b - 'a' + 0xa;
            else if(*b >= 'A' && *b <= 'F')
              u += *b - 'A' + 0xa;
          }
          convert_ucs2_to_utf8(u, s);
        }
        break;
      default:
        s.push_back(*b);
        break;
      }
      break;
    default:
      s.push_back(*b);
      break;
    }
  }
  throw Error();
}

Value parse_number(Reader& b) {
  double i = 0.0;
  double x = 1.0;
  double s = 1.0;

  if(!b) throw Error();

  if(*b == '-') {
    s = -1.0;
    ++b;
    if(!b) throw Error();
  }

  if(!isdigit(*b)) throw Error();

  for(;b && isdigit(*b); ++b)
    i = i*10 + (*b - '0');

  if(b) {
    if('.' == *b) {
      ++b;
      if(!b || !isdigit(*b)) throw Error();
      for(double m = 0.1; b && isdigit(*b); m /=10, ++b)
        i += (*b - '0') * m;
    }
  }

  if(b) {
    if(*b == 'e' || *b == 'E') {
      ++b;
      if(!b) throw Error();
      
      bool se = false;
      if(*b == '+') {
        ++b;
        if(!b) throw Error();
      }
      else if(*b == '-') {
        ++b;
        if(!b) throw Error();
        se = true;
      }
      if(!isdigit(*b)) throw Error();

      for(x=0; b && isdigit(*b); ++b)
        x = x*10 + (*b - '0');
      if(se) x = 1/x;
    }
  } 
  
  return s*i*x;
}

void parse_literal(Reader& b, char const* s) {
  for(;*s;) {
    if(!b || *b != *s) throw Error();
    ++s;
    ++b;
  }
}

Value parse(Reader& b) {
  skipws(b);

  if(!b) return null;
  
  switch(*b) {
  case '{':
    return parse_object(b);
  case '[':
    return parse_array(b);
  case '"':
    return parse_string(b);
  case 't': 
    parse_literal(b, "true");
    return true;
  case 'f': 
    parse_literal(b, "false");
    return false;
  case 'n': 
    parse_literal(b, "null");
    return null;
  default:
    return parse_number(b);
  }
}

} // Detail

Value parse(std::istream& is) {
  Detail::Reader b(is);
  return Detail::parse(b);
}

}

