// header-start
////////////////////////////////////////////////////////////////////////////////
// \project   all
//
// \file      util.h
//
////////////////////////////////////////////////////////////////////////////////
// header-end


#pragma once

#include <string>
#include <sstream>
#include <type_traits>
#include <fmt/format.h>

namespace UtilOld {

//
//! \brief A templatized alignment to 32 bytes boundary
//!

template<typename Type>
Type *addr_align32(const Type *storage) {
  uintptr_t tmp = reinterpret_cast<uintptr_t>(storage);
  uintptr_t mask = 0x1f;

  if (tmp & mask) {    // some bits [4:0] are set
    tmp = (tmp | mask) + 1;
  }
  return reinterpret_cast<Type *>(tmp);
}

//
// Clamp class functor
//
template<typename tpl_t>
class Clamp {
 private:
  tpl_t min_;
  tpl_t max_;

 public:
  tpl_t operator()(tpl_t data) {
    if (std::is_unsigned<tpl_t>::value && min_ == 0) {
      return (data > max_) ? max_ : data;
    } else {
      return (data < min_) ? min_ : ((data > max_) ? max_ : data);
    }
  }
  Clamp(tpl_t min, tpl_t max) : min_{min}, max_{max} {
  }
};

//
// Commify class functor
// thousand digit separator
// 1234 => 1,234
//


class Commify {
 private:
  std::string str_;
  void insert_separator(char c) {
    auto pos = str_.length();
    while (pos > 3) {
      pos -= 3;
      str_.insert(pos, 1, c);
    }
  }
 public:
  explicit Commify(uint64_t value) {
    std::ostringstream ss;
    ss << value;
    str_ = ss.str();
    insert_separator(',');
  }

  explicit Commify(int value) {
    std::ostringstream ss;
    ss << value;
    str_ = ss.str();
    insert_separator(',');
  }
  friend std::ostream& operator<<(std::ostream &os, const Commify &c) {
    os << c.str_;
    return os;
  }

  std::string& str() {
    return str_;
  }
};

//! \brief
//! See https://en.wikipedia.org/wiki/ANSI_escape_code#CSI_codes
//! This class is implemented as a functor
//! Example:  cout << "normal text" << CSI("italic") << "italic text" << CSI("normal");

class CSI {
 private:
  std::string str_;

 public:
  explicit CSI(std::string value) {
    std::ostringstream ss;
    if (value == "black") {
      ss << "\033[30m";
    } else if (value == "red") {
      ss << "\033[31m";
    } else if (value == "green") {
      ss << "\033[32m";
    } else if (value == "yellow") {
      ss << "\033[33m";
    } else if (value == "blue") {
      ss << "\033[34m";
    } else if (value == "magenta") {
      ss << "\033[35m";
    } else if (value == "cyan") {
      ss << "\033[36m";
    } else if (value == "default") {
      ss << "\033[0m";
    } else if (value == "italic") {
      ss << "\033[3m";
    } else if (value == "bold") {
      ss << "\033[1m";
    } else {
      ss << "\033[0m";
    }
    str_ = ss.str();
  }
  friend std::ostream& operator<<(std::ostream &os, const CSI &c) {
    os << c.str_;
    return os;
  }
};


}  // namespace Util

struct Util {

  static std::string
  commify(uint64_t value) {
    std::string res = fmt::format("{:d}", value);
    char c = ',';
    auto pos = res.length();
    while (pos > 3) {
      pos -= 3;
      res.insert(pos, 1, c);
    }
    return res;
  }

  static std::string
  csi(std::string value) {
    std::string ss;
    if (value == "black") {
      ss = "\033[30m";
    } else if (value == "red") {
      ss = "\033[31m";
    } else if (value == "green") {
      ss = "\033[32m";
    } else if (value == "yellow") {
      ss = "\033[33m";
    } else if (value == "blue") {
      ss = "\033[34m";
    } else if (value == "magenta") {
      ss = "\033[35m";
    } else if (value == "cyan") {
      ss = "\033[36m";
    } else if (value == "default") {
      ss = "\033[0m";
    } else if (value == "italic") {
      ss = "\033[3m";
    } else if (value == "bold") {
      ss = "\033[1m";
    } else {
      ss = "\033[0m";
    }
    return ss;
  }
};
