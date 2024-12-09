#pragma once

#include <iostream>
#include <iomanip>
#include <ctime>
#include <string>
#include <string_view>
#include <sstream>
#include <type_traits>
#include <fmt/format.h>
#include <utility>
#include <locale>
#include <source_location>


// 2024-03-12T19:29:10GMT [INFO]   mytext.check()
// 2024-03-12T19:29:10GMT [DEBUG]  mytext.check()
// 2024-03-12T19:29:10GMT [TRACE]  mytext.check()
// 2024-03-12T19:29:10GMT []  mytext.check()

class logger {
 private:
  static void generic(const std::string& what, const std::string& str) {
    auto t = std::time(nullptr);
    auto tm = *std::gmtime(&t);
    std::string type_str = fmt::format("[{}]", what);
    std::cout << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S%Z") << fmt::format(" {0:7} {1}\n",type_str, str);
  }

  inline static bool enable_debug_ = false;

 public:

  static void enable_debug() {
    logger::enable_debug_ = true;
  }

  static void disable_debug() {
    logger::enable_debug_ = false;
  }

  // INFO
  template<typename... Args>
  static void info(const std::string& frmt, Args&&... args) {
    std::string_view frmt_view{frmt};
    std::string astr = fmt::format(fmt::runtime(frmt), args...);
    logger::generic("INFO", astr);
  }

  // WARN
  template<typename... Args>
  static void warn(const std::string& frmt, Args&&... args) {
    std::string_view frmt_view{frmt};
    std::string astr = fmt::format(fmt::runtime(frmt), args...);
    logger::generic("WARN", astr);
  }
  
  // ERROR
  template<typename... Args>
  static void error(const std::string& frmt, Args&&... args) {
    std::string_view frmt_view{frmt};
    std::string astr = fmt::format(fmt::runtime(frmt), args...);
    logger::generic("ERROR", astr);
  }
  

  // DEBUG
  template<typename... Args>
  static void debug(const std::string& frmt, Args&&... args) {
    if (logger::enable_debug_) {
      std::string_view frmt_view{frmt};
      std::string astr = fmt::format(fmt::runtime(frmt), args...);
      logger::generic("DEBUG", astr);
    }
  }

  // TRACE
  template<typename... Args>
  static void trace(const std::string& frmt, Args&&... args) {
    std::string_view frmt_view{frmt};
    std::string astr = fmt::format(fmt::runtime(frmt), args...);
    logger::generic("TRACE", astr);
  }

  // FATAL
  // no variadic here
  static void fatal(const std::string& err_msg, std::source_location location = std::source_location::current())
  {
    std::string astr = fmt::format("Exception in function {} at line {}",location.function_name(), location.line());
    logger::generic("FATAL", err_msg);
    logger::generic("FATAL", astr);
  }
  

};
