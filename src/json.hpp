#include <string>
#include <map>
#include <vector>
#include <boost/variant.hpp>

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

Value parse(char const*& begin, char const* end);
std::string parseString(char const*& begin, char const* end);

Value parseObject(char const*& b, char const* e) {
  std::map<std::string, Value> o;
 
  ++b; 
  if(*b == '}') {
     ++b;
     return o;
  }
 
  while(b != e) {
       std::string name = parseString(b,e);

    if(*b != ':') throw Error();
    ++b;

    o[name] = parse(b,e);

    if(*b == '}') {
      ++b;
      return o;
    }
    else if(*b != ',')
      throw Error();
    ++b;
  }

  throw Error();
}

Value parseArray(char const*& b, char const* e) {
  std::vector<Value> v;

  ++b;
  while(b != e) {
    if(*b == ']') {
      ++b;
      return v;
    }
    
    v.push_back(parse(b, e));

    if(b == e)
      throw Error();

    if(*b == ']') {
      ++b;
      return v;
    }

    if(*b != ',') throw Error();
    ++b;
  }

  throw Error();
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

std::string parseString(char const*& b, char const* e) {
  std::string s;
  
  if(*b != '"') throw Error();

  for(++b;b != e;++b) {
    switch(*b) {
    case '"':
      ++b;
      return s;
    case '\\':
      ++b;
      if(b == e) throw Error();
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
      case 'u':
        if(e-b < 5)
          throw Error();
        else { 
          ++b;
          int u = 0;
          for(int i = 0; i < 4; ++i) {
            u *= 16;
            if(b[i] >= '0' && b[i] <= '9')
              u += b[i] - '0';
            else if(b[i] >= 'a' && b[i] <= 'f')
              u += b[i] - 'a';
            else if(b[i] >= 'A' && b[i] <= 'F')
              u += b[i] - 'A';
          }
          b += 4;
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

Value parseNumber(char const*& b, char const* e) {
  double i = 0.0;
  double x = 1.0;
  double s = 1.0;

  if(*b == '-') {
    s = -1.0;
    ++b;
    if(e == b) throw Error();
  }

  if(!isdigit(*b)) throw Error();

  for(;b != e && isdigit(*b); ++b)
    i = i*10 + (*b - '0');

  if(b != e) {
    if('.' == *b) {
      ++b;
      if(e == b || !isdigit(*b)) throw Error();
      for(double m = 0.1; b !=e && isdigit(*b); m /=10, ++b)
        i += (*b - '0') * m;
    }
  }

  if(b != e) {
    if(*b == 'e' || *b == 'E') {
      ++b;
      if(e == b) throw Error();
      
      bool se = false;
      if(*b == '+') {
        ++b;
        if(e == b) throw Error();
      }
      else if(*b == '-') {
        ++b;
        if(e == b) throw Error();
        se = true;
      }
      if(!isdigit(*b)) throw Error();

      for(x=0; b!=e && isdigit(*b); ++b)
        x = x*10 + (*b - '0');
      if(se) x = 1/x;
    }
  } 
  
  return s*i*x;
}

bool is_literal(char const*& begin, char const* e, char const* s) {
  char const* b = begin;
  for(;b != e && *s != 0;)
    if(*b != *s) return false;

  begin = b;
  return true;
}

Value parse(char const*& b, char const* e) {
  if(e - b == 0)
    throw Error();

  if(*b == '{') {
    return parseObject(b, e);
  }
  else if(*b == '[') {
    return parseArray(b, e);
  }
  else if(*b == '"') {
    return parseString(b, e);
  }
  else if(is_literal(b, e, "true")) {
    return true;
  }
  else if(is_literal(b, e, "false")) {
    return false;
  }
  else if(is_literal(b, e, "null")) {
    return null;
  }
  else {
    return parseNumber(b, e);
  }
}

Value parse(std::string const& s) {
  char const* b = &s[0];
  return parse(b, &s[0]+s.length());
}

}
