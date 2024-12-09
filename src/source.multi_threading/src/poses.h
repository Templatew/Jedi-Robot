// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      poses.h
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#pragma once

#include <ostream>
#include <tuple>
#include <string>
#include <vector>
#include <glm/mat4x4.hpp> // mat4


class Pose
{
 private:

  std::vector<std::tuple<std::string, glm::mat4, bool, bool >> link_transforms_;
  
 public:
  Pose() = default;
  std::vector<std::pair<std::string, float>> joint_angles_;
  void add_joint_angle(const std::string& name, float);
  void add_link_transform(const std::string& name, const glm::mat4& transform, bool is_root, bool is_target);
  float get_joint_angle(const std::string& name) const;
  const glm::mat4& get_link_transform(const std::string& name) const;
  void write_blender_script(std::ofstream& ostrm) const;
  void write_blender_animation_script(std::ofstream& ostrm, int framenumber) const;
  void write_blender_select_target(std::ofstream& ostrm) const;
};

class Poses
{
 private:
  std::vector<Pose> poses_;
  
 public:
  Poses() = default;
  void add_pose(const Pose& pose) {
    poses_.push_back(pose);
  }
  Pose pop_front();     // the first one
  Pose pop_back();  // the last one
  void write_blender_script(const std::string& filename) const;
  Pose get_first_pose() const { return poses_.front(); }
  Pose get_last_pose() const { return poses_.back(); }
};
