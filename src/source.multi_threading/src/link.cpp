// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      joint.cpp
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#include "link.h"

#include <glm/mat4x4.hpp>
#include <glm/trigonometric.hpp>
#include <glm/vec3.hpp>
#include <iostream>
#include <string>
#include <vector>

Link::Link(const std::string& name, const std::string& mesh_name)
    : name_{name}, mesh_name_{mesh_name}, global_transform_{glm::mat4(1.0f)}
{
  is_target_ = false;
  is_root_ = false;
  p_parent_link_ = nullptr;
}


void
Link::foreach_child_joint(std::function<void(Joint*)> fn)
{
  for (auto& pjoint : pjoints_) {
    fn(pjoint);
  }
}

bool
Link::is_root() const
{ 
  return is_root_;
}

bool
Link::is_leaf() const
{ 
  return pjoints_.size() == 0;
}

void
Link::info() const
{
  std::cout << "Info, link name   = " << name_ << '\n';
  if (is_root()) {
    std::cout << "Info, link is root" << '\n';
  } else {
    std::cout << "Info, link parent = " << (parent_link()->name()) << '\n';
  }
  if (is_leaf()) {
    std::cout << "Info, link is leaf" << '\n';
  } else {
    std::cout << "Info, link number of children = "
              << pjoints_.size() << '\n';
  }
  std::cout << "Info, global transform = " << global_transform_ << '\n';
  std::vector<float> xyz_rpy = glmutil::get_xyz_rpy(global_transform_);
  std::cout << "Info Translation xyz = (" << xyz_rpy[0] << ", " << xyz_rpy[1]
            << ", " << xyz_rpy[2] << ")\n";
  std::cout << "Info Rotation rpy  = (" << glm::degrees(xyz_rpy[3]) << ", "
            << glm::degrees(xyz_rpy[4]) << ", " << glm::degrees(xyz_rpy[5])
            << ")\n";
}

