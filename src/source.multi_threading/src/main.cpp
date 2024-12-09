// header-start
////////////////////////////////////////////////////////////////////////////////
// \file      main.cpp
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <exception>
#include <glm/trigonometric.hpp>
#include <glm/vec3.hpp>
#include <iostream>
#include <map>
#include <random>
#include <regex>
#include <string>
#include <toml.hpp>
#include <vector>
#include <filesystem>

#include <thread>
#include <mutex>
#include <vector>
#include <atomic>

#include "logger.h"
#include "robot.h"
#include "util.h"
#include "poses.h"
#include "context.h"
#include "ikparams.h"

#ifndef PROG_NAME
const std::string PROG_NAME{"ik.exe"};
#endif

#ifndef PROG_VERSION
const std::string PROG_VERSION{"0.00"};
#endif

#ifndef PROG_GITID
const std::string PROG_GITID{"????+0"};
#endif

#ifdef NDEBUG
const std::string COMPILE_MODE{"Release"};
#else
const std::string COMPILE_MODE{"Debug  "};
#endif

// Global variables
std::string robot_name_g;
toml::value top_level_data_g;

///////////////////////////////////////////////////////////////////////////////
//
// Class MainConfig is a possible  way of dealing with console based
// option: it perform all the options processing and checking
//
//

struct MainConfig {
  MainConfig(const int argc, const char* const argv[]);
  void print_version() const;
  void print_help() const;
  void print_banner() const;
  Robot create_robot();
  void check_valid_entries(toml::value& top_level_data);
  void processing(Robot& robot);
  void parse_list_of_numbers(const std::string& current_arg,
                             const std::string& values);
  glm::vec3 get_vec3(const std::string& joint_name,
                     toml::table& a_table,
                     const std::string& a_key);
  float get_float(const std::string& joint_name,
                  toml::table& a_table,
                  const std::string& a_key);

  const std::vector<std::vector<float>>& get_angles() const { return angles_; }
  const std::string& blender_filename() const { return output_blender_script_; }

  void interpolation(Robot& robot, Poses& poses, Pose& start_pose, Pose& end_pose, int steps);

  std::string program_name_;
  std::string program_version_;
  std::string program_gitid_;
  std::string program_option_;

  // argument parameters

  std::string ik_method_;
  std::string output_file_name_;
  std::string config_file_name_;
  std::string output_blender_script_;

  size_t nb_threads_;
  size_t nb_iterations_;
  uint32_t seed_;

  std::vector<std::vector<float>> angles_;
  std::vector<float> xyz_;
  std::vector<float> rpy_;
};

void
MainConfig::check_valid_entries(toml::value& top_level_data)
{
  std::map<std::string, decltype(toml::value_t::table)> valid_entries{
      {"info", toml::value_t::table},
      {"links", toml::value_t::array},
      {"joints", toml::value_t::array}};
  for (const auto& an_entry : valid_entries) {
    auto& [a_key, a_type] = an_entry;
    if (not top_level_data.contains(a_key)) {
      throw std::domain_error(
          fmt::format("Variable '{}' not found in config file", a_key));
    }
    if (top_level_data.at(a_key).type() != a_type) {
      throw std::domain_error(
          fmt::format("Variable '{}' has wrong type", a_key));
    }
  }
}

Robot
MainConfig::create_robot()
{
  try {

    if (Robot::robot_count == 0){
      //
      // parse and check the robot config file
      //
      logger::info("Parsing config file {0}", config_file_name_);
      toml::value top_level_data = toml::parse(config_file_name_);
      top_level_data_g = top_level_data;
      check_valid_entries(top_level_data);

      //
      // create the robot
      //

      auto& tbl_info = toml::find(top_level_data, "info");
      robot_name_g = toml::find<std::string>(tbl_info, "robot_name");
      logger::info("Creating robot {0}", robot_name_g);

      const auto dir_name = toml::find<std::string>(tbl_info, "root_path");
      auto dir_path = std::filesystem::path(dir_name);
      if (not std::filesystem::is_directory(dir_path)) {
        logger::error("The 'root_path' entry '{0}' is not a valid directory", dir_name);
        std::exit(1);
      }

    }

    Robot robot(robot_name_g, top_level_data_g);
    return robot;

  } catch (const toml::syntax_error& error) {
    logger::fatal(error.what());
    std::exit(1);
  } catch (const std::exception& error) {
    logger::fatal(error.what());
    std::exit(1);
  }
}

//
// print banner
//
//                     1         2         3         4         5
void
MainConfig::print_banner() const
{
  //const std::string compile_date = __DATE__;
  const std::string compile_time = __TIMESTAMP__;
  const std::string title = fmt::format("Forward and Inverse Kinematic ({} version {})", program_option_, program_version_);
  const std::string config = fmt::format("Compiled on {} from source ID {}", compile_time, program_gitid_);

  int w{76};
  fmt::print("##{}##\n", std::string(w, '#'));
  fmt::print("##{}##\n", std::string(w, ' '));
  fmt::print("##{:^{}}##\n", title, w);
  fmt::print("##{:^{}}##\n", config, w);
  fmt::print("##{}##\n", std::string(w, ' '));
  fmt::print("##{}##\n", std::string(w, '#'));
}

void
MainConfig::print_version() const
{
  std::cout << program_name_ << " " << program_version_ << std::endl;
}

void
MainConfig::print_help() const
{
  fmt::print("Usage: {0} [options]\n", program_name_);
  fmt::print("Options:\n");

  std::vector<std::array<std::string, 3>> options = {
      {"-h | --help",  "",            "Display this help"},
      {"--version",    ""  ,          "Display the program version"},
      {"--config",     "filename",    "Name of the input robot config file in toml format, default to config.toml"},
      {"--blender",    "filename",    "Name of the output blender script, default to my_script.py"},
      {"--angles",     "float,...",   "The angles of the joints in degrees for FK computation, can be repeated"},
      {"--xyz",        "float,...",   "The target position for IK computation, 3 values"},
      {"--rpy",        "float,...",   "The target rotation for IK computation, 3 values"},
      {"--method",     "string",      "Extra option to enable variants in CCD IK computation"},
      {"--iterations", "integer",     "Number of iterations for the CCD IK computation"},
      {"--seed",       "integer",     "Force the seed used by the random number generator"}};

  for (const auto& option : options) {
    fmt::print("{0:<13}{1}{2:<12}{3}{4}\n", option[0], Util::csi("italic"),
               option[1], Util::csi("normal"), option[2]);
  }
}

//
// angles are in degree

void
MainConfig::parse_list_of_numbers(const std::string& current_arg, const std::string& values)
{
  std::vector<float> angles;

  try {
    // split string with ,

    std::regex sep(",");
    std::sregex_token_iterator it{values.cbegin(), values.cend(), sep, -1};
    std::sregex_token_iterator end;

    for (; it != end; ++it) {
      std::string sub_str = *it;
      float avalue = std::stof(sub_str);
      if (current_arg == "--angles") {
        angles.push_back(glm::radians(avalue));
      } else if (current_arg == "--xyz") {
        xyz_.push_back(avalue);
      } else if (current_arg == "--rpy") {
        rpy_.push_back(glm::radians(avalue));
      } else {
        throw robot::exception("Runtime error '{}' is not an expected value", current_arg);
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "Error: argument " << current_arg << " requires a number";
    std::cerr << ", but got " << values << std::endl;
    std::cerr << "Error code: " << error.what() << std::endl;
    std::exit(1);
  }
  if (current_arg == "--angles") {
    angles_.push_back(std::move(angles));
  }
}

//
//
//
MainConfig::MainConfig(const int argc, const char* const argv[])
{
  // default parameters values
 
  //nb_threads_ = 1;
  nb_threads_ = std::thread::hardware_concurrency();
  program_name_ = PROG_NAME;
  program_version_ = PROG_VERSION;
  program_gitid_ = PROG_GITID;
  program_option_ = COMPILE_MODE;

  output_file_name_ = "results.txt";
  output_blender_script_ = "my_script.py";
  config_file_name_ = "config.toml";

  nb_iterations_ = 100;
  seed_ = std::random_device{}();

  for (int idx = 1; idx < argc; ++idx) {
    std::string current_arg = argv[idx];
    if (current_arg == "-h" || current_arg == "--help") {
      print_help();
      std::exit(0);
    }
    if (current_arg == "--version") {
      print_version();
      std::exit(0);
    }
    if (current_arg == "--debug") {
      logger::enable_debug();
      continue;
    }

    // from this point on all arguments
    // have one parameter

    idx++;
    if (idx >= argc) {
      std::cerr << "Error: argument " << current_arg
                << " needs a value, or unknown" << std::endl;
      std::exit(1);
    }
    std::string current_value{argv[idx]};

    if (current_arg == "--output") {
      output_file_name_ = argv[idx];
      continue;
    }

    if (current_arg == "--blender") {
      output_blender_script_ = argv[idx];
      continue;
    }

    if (current_arg == "--config") {
      config_file_name_ = argv[idx];
      continue;
    }

    if ((current_arg == "--iterations") or
        (current_arg == "--seed")) {
      size_t v;
      try {
        v = std::stoul(current_value);
      } catch (const std::exception& error) {
        std::cerr << "Error: argument " << current_arg << " requires a number";
        std::cerr << ", but got " << current_value << std::endl;
        std::exit(1);
      }
      if (current_arg == "--seed") {
      seed_ = v;
      } else {
        nb_iterations_ = v;
      }
      continue;
    }

    if ((current_arg == "--xyz") or
        (current_arg == "--rpy") or
        (current_arg == "--angles")) {
      parse_list_of_numbers(current_arg, current_value);
      continue;
    }

    // free text argument to be passed
    // to the IK algorithm
    if (current_arg == "--method") {
      ik_method_ = argv[idx];
      continue;
    }
  }
}


void MainConfig::interpolation(Robot& robot, Poses& poses, Pose& start_pose, Pose& end_pose, int steps) {
  
  poses.add_pose(start_pose);

  for (int i = 0; i <= steps; ++i) {
    float t = static_cast<float>(i) / steps;

    // Create an interpolated pose
    Pose interpolated_pose;
    std::vector<float> angles;

    // Interpolate angles and add the next interpolated pose angles to the angles vector
    for (const auto& [name, angle] : start_pose.joint_angles_) {
      float start_angle = start_pose.get_joint_angle(name);
      float end_angle = end_pose.get_joint_angle(name);
      float interpolated_angle = glm::mix(start_angle, end_angle, t);
      angles.push_back(interpolated_angle);
    }

    // Compute forward kinematics
    robot.forward_kinematic(angles);
    interpolated_pose = robot.get_current_pose();
    poses.add_pose(interpolated_pose);
  }
}

int
main(int argc, char* argv[])
{

  MainConfig main_config(argc, argv);
  main_config.print_banner();

  std::vector<Robot> robots; // Holds a robot instance for each thread
  auto start_creation_time = std::chrono::high_resolution_clock::now();

  for (size_t i = 0; i < main_config.nb_threads_; ++i) {
    robots.push_back(main_config.create_robot());
  }

  auto end_creation_time = std::chrono::high_resolution_clock::now();
  auto creation_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_creation_time - start_creation_time).count();
  logger::info("Robots creation time: {} ms", creation_elapsed_time);

  // Recording the robot position to be used
  // to generate script file
  // (example to a blender python script)

  Poses poses;


  // forward kinematic check parameters

  for(const auto& angles: main_config.get_angles()) {
    if (angles.size() != robots[0].degrees_of_freedom()) {
      logger::error("Incorrect number of angles for the angle parameters, got {}, but 0 or {} expected",
        angles.size(), robots[0].degrees_of_freedom());
      std::exit(-1);
    }
  }

  if (main_config.angles_.size() > 0) {
    logger::info("---------------------------------");
    logger::info("--   Start Forward Kinematic   --");
    logger::info("---------------------------------");

    for(const auto& angles : main_config.get_angles()) {
      logger::info("---------------------------------");
      auto result = robots[0].forward_kinematic(angles, true);
      if (not result) {
        logger::error("Cannot compute forward kinematic, quit now");
        std::exit(-1);
      }
      auto pose = robots[0].get_current_pose();
      poses.add_pose(pose);
    }
  }

  // inverse kinematic, check parameters
 
  if (not((main_config.xyz_.size() == 3 and main_config.rpy_.size() == 3) or
          (main_config.xyz_.size() == 0 and main_config.rpy_.size() == 0))) {
    logger::error("Incorrect value for the IK parameters (xyz or rpy)");
    std::exit(-1);
  }

  if (main_config.xyz_.size() == 3 and main_config.rpy_.size() == 3) {

    logger::info("---------------------------------");
    logger::info("--   Start Inverse Kinematic   --");
    logger::info("---------------------------------");
    logger::info("Using seed {}", main_config.seed_);

    
    std::vector<float> rpy_in_degrees;
    for(const auto& value: main_config.rpy_) {
      rpy_in_degrees.push_back(glm::degrees(value));
    }

    logger::info("Target position is ({:g})", fmt::join(main_config.xyz_, ","));
    logger::info("Target rotation is ({:g})", fmt::join(rpy_in_degrees, ","));

    float distance_threshold = 0.01f;
    int angle_res_bits = 16;

    logger::info("Distance threshold: {}", distance_threshold);
    logger::info("Angle resolution bits: {}", angle_res_bits);

    logger::info("---------------------------------");
    logger::info("--           Threads           --");
    logger::info("---------------------------------");
    logger::info("Number of threads: {}", main_config.nb_threads_);

    size_t num_threads = main_config.nb_threads_;
    std::vector<float> distances(num_threads);
    float best_distance = std::numeric_limits<float>::max();
    size_t best_robot_index = 0;
    std::atomic<bool> stop_flag(false);
    size_t threads_tasks = 10000;
    std::mutex mutex;
    std::atomic<size_t> runs_completed(0);
    std::vector<std::thread> threads;

    auto start_time = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < main_config.nb_threads_; ++i) {
      threads.emplace_back([&, i]() {
        Robot& robot = robots[i];

        while (!stop_flag.load()) {
          size_t run_index = runs_completed.fetch_add(1);
          if (run_index >= threads_tasks || stop_flag.load()) {
            break;
          }

          IKParams ik_params(
            main_config.xyz_,
            main_config.rpy_,
            main_config.seed_ + static_cast<uint32_t>(run_index),
            main_config.nb_iterations_ * robot.degrees_of_freedom(), // times the number of joints
            main_config.ik_method_);
          
          float distance = robot.inverse_kinematic(ik_params, distance_threshold, 16);

          {
            std::lock_guard<std::mutex> lock(mutex);

            if (run_index % 100 == 0) {
            logger::info("Thread task nb: {}, current best: {}", run_index, best_distance);
            }

            if (distance < best_distance) {
              best_distance = distance;
              best_robot_index = i;
            }

            if (distance < distance_threshold) {
              if (stop_flag.load()) {
                break;
              }
              stop_flag.store(true);
              if (i < 10) {
                logger::info("Thread  {} succeeded, distance: {}", i, distance);
              } 
              else {
              logger::info("Thread {} succeeded, distance: {}", i, distance);
              }
            }
          }

          if (stop_flag.load()) {
            break;
          }

        }
      });
    }

    // Wait for threads to finish
    for (auto& thread : threads) {
      if (thread.joinable()) {
        thread.join();
      }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    Robot best_robot = robots[best_robot_index];

    Pose best_pose = best_robot.get_current_pose();

    logger::info("---------------------------------");
    logger::info("--           Results           --");
    logger::info("---------------------------------");

    logger::info("Reached Target: {}", stop_flag.load());
    logger::info("Final distance: {}", best_distance);
    logger::info("Elapsed time: {} ms", elapsed_time);

    std::vector<float> angles_in_degrees;
    for(const auto& joint: best_robot.get_joints()) {
      if (joint.has_degree_of_freedom()) {
        angles_in_degrees.push_back(glm::degrees(joint.get_angle()));
      }
    }
    logger::info("Joint angles are ({:g})", fmt::join(angles_in_degrees,","));
    
    // Interpolation between forward and inverse kinematics poses
    int steps = 188;
    Pose first_pose = poses.get_first_pose();
    main_config.interpolation(robots[0], poses, first_pose, best_pose, steps);

    /*std::vector<std::vector<float>> angles_test = {
      {-20,-65,48,37,-25,20},
      {80,-65,0,-20,60,180},
      {0,0,20,20,60,-150}
    };

    for (auto& angles : angles_test) {
      for (auto& angle : angles) {
        angle = glm::radians(angle);
      }
      robots[0].forward_kinematic(angles, false);
      auto pose = robots[0].get_current_pose();
      Pose last_pose = poses.get_last_pose();
      main_config.interpolation(robots[0], poses, last_pose, pose, steps);
    }*/
    
    //Pose last_pose = poses.get_last_pose();
    //main_config.interpolation(robots[0], poses, last_pose, first_pose, steps);
  }

  logger::info("---------------------------------");
  logger::info("--    Write Blender Script     --");
  logger::info("---------------------------------");

  poses.write_blender_script(main_config.blender_filename());

  return 0;
}
