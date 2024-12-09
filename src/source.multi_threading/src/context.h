// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      context.h
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//


#include <unistd.h>
#include <limits.h>
#include <iostream>
#include <string>
#include <exception>
#include <fmt/format.h>

#include "logger.h"


class Robot;
class Poses;


//
// RunContext is a singleton class 
// to store global information
//

class RunContext {
 private:
  Robot* robot_;       // non-owning
  Poses *poses_;       // non-owning

  std::string hostname_;
  std::string username_;


  RunContext()
  {
    robot_ = nullptr;
    poses_ = nullptr;

    char hostname[HOST_NAME_MAX];
    char username[LOGIN_NAME_MAX];
    int result;
    result = gethostname(hostname, HOST_NAME_MAX);
    if (result) {
      hostname_ = "HostNameUnknown";
    } else {
      hostname_ = hostname;
    }

    result = getlogin_r(username, LOGIN_NAME_MAX);
    if (result) {
      username_ = "UserNameUnknown";
    } else {
      username_ = username;
    }
  }

  ~RunContext() {
    //std::cout << "DEBUG: Singleton \"RunContext\" call" << std::endl;
  }
 public:
  static RunContext &instance() {
    static RunContext obj;
    return obj;
  }

  std::string& get_username() {
    return username_;
  }

  std::string& get_hostname() {
    return hostname_;
  }

  void
  set_robot(Robot *robot) {
    robot_ = robot;
  }

  Robot* get_robot() {
    return robot_;
  }

  void
  set_poses(Poses *poses) {
    poses_ = poses;
  }

  Poses* get_poses() {
    return poses_;
  }
};


class robot {
 public:
  class exception: public std::exception {
   private:
    std::string message_;
   public:
    const char *what() const noexcept override {
      return message_.data();
    }

    template<typename... Args>
    exception(const std::string& frmt, Args&&... args) {
      std::string_view frmt_view{frmt};
      message_ = fmt::format(fmt::runtime(frmt), args...);
    }
  };
};


