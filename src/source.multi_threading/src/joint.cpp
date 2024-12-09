// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      joint.cpp
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//


#include <fmt/format.h>

#include <algorithm>
#include <glm/gtc/matrix_access.hpp>  // need to be included to use these features.
#include <glm/mat4x4.hpp>             // mat4
#include <glm/trigonometric.hpp>      //radians
#include <glm/vec3.hpp>               // vec3
#include <iostream>
#include <numbers>
#include <string>

#include "logger.h"
#include "joint.h"


std::ostream&
operator<<(std::ostream& os, const JointType& jt)
{
  if (jt == JointType::fixed) {
    os << "fixed";
  } else if (jt == JointType::continuous) {
    os << "continuous";
  } else if (jt == JointType::revolute) {
    os << "revolute";
  } else {
    os << "unknown";
  }
  return os;
}

Joint::Joint(const std::string& name)
    : name_{name},
      local_xyz_{0.0f, 0.0f, 1.0f},
      local_rpy_{0.0f, 0.0f, 1.0f},
      local_translation_{glm::mat4(1.0f)},
      local_rotation_{glm::mat4(1.0f)},
      joint_angle_transform_{glm::mat4(1.0f)},
      local_transform_{glm::mat4(1.0f)}
{
  angle_ = 0.0f;
  joint_type_ = JointType::fixed;
  idx_nonfixed_ = std::numeric_limits<size_t>::max(); 
  min_angle_ = 0.0f;
  max_angle_ = 0.0f;
  p_parent_link_ = nullptr;
  p_child_link_ = nullptr;
}

void
Joint::info()
{
  std::cout << "Joint::info() name =" << name_ << "\n";
  std::cout << "Joint::info() type =" << joint_type_ << "\n";
  std::cout << "Joint::info() angle_ =" << angle_ << "\n";
  std::cout << "Joint::info() min_angle_ =" << min_angle_ << "\n";
  std::cout << "Joint::info() max_angle_ =" << max_angle_ << "\n";
}

void 
Joint::set_joint_type(const std::string& joint_type, float min_angle, float max_angle)
{
  if (joint_type == "fixed") {
    joint_type_ = JointType::fixed;
    min_angle_ = 0.0f;
    max_angle_ = 0.0f;
  } else if (joint_type == "continuous") {
    joint_type_ = JointType::continuous;
    min_angle_ = -(std::numbers::pi_v<float>);
    max_angle_ = std::numbers::pi_v<float>;
  } else if (joint_type == "revolute") {
    joint_type_ = JointType::revolute;

    if (min_angle_ >= max_angle) {
      std::cerr << "Illegal min/max angle" << std::endl;
      exit(-1);
    }
    min_angle_ = std::max<float>(min_angle, -(std::numbers::pi_v<float>));
    max_angle_ = std::min<float>(max_angle, std::numbers::pi_v<float>);
  } else {
    std::cerr << "Un-supported joint type " << joint_type << std::endl;
    exit(-1);
  }
}

void
Joint::set_local(const glm::vec3& t_xyz, const glm::vec3& r_rpy)
{
  local_xyz_ = t_xyz;
  local_rpy_ = r_rpy;
  glm::vec4 t4_xyz{t_xyz.x, t_xyz.y, t_xyz.z, 1.0f};

  local_translation_ = glm::column(glm::mat4(1.0), 3, t4_xyz);

  // roll
  float cx = std::cos(r_rpy.x);
  float sx = std::sin(r_rpy.x);

  // layout appear to be correct for a rotation
  // but must be transposed as glm is in col major

  glm::mat4 rx{{1, 0, 0, 0}, {0, cx, -sx, 0}, {0, sx, cx, 0}, {0, 0, 0, 1}};

  rx = glm::transpose(rx);

  // pitch
  float cy = std::cos(r_rpy.y);
  float sy = std::sin(r_rpy.y);

  glm::mat4 ry{{cy, 0, sy, 0}, {0, 1, 0, 0}, {-sy, 0, cy, 0}, {0, 0, 0, 1}};

  ry = glm::transpose(ry);

  // yaw
  float cz = std::cos(r_rpy.z);
  float sz = std::sin(r_rpy.z);

  glm::mat4 rz{{cz, -sz, 0, 0}, {sz, cz, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  rz = glm::transpose(rz);

  local_rotation_ = rz * ry * rx;
  local_transform_ = local_translation_ * local_rotation_ * joint_angle_transform_;
}


void
Joint::idx_nonfixed(size_t idx)
{
  if (not has_degree_of_freedom()) {
    logger::warn("Cannot set index on fixed joint '{}'", name());
    return;
  }
  idx_nonfixed_ = idx;
}

size_t
Joint::idx_nonfixed(void)
{
  if (not has_degree_of_freedom()) {
    logger::warn("Cannot get index on fixed joint '{}'", name());
  }
  return idx_nonfixed_;
}

void
Joint::set_angle(const float angle)
{
  if (not has_degree_of_freedom()) {
    logger::warn("Angle cannot be set on fixed joint '{}'", name());
    return;
  }

  // assume angle on axis (0,0,1)
  // yaw
  if (angle < min_angle_) {
    angle_ = min_angle_;
  }
  if (angle > max_angle_) {
    angle_ = max_angle_;
  }
  angle_ = angle;
  float cz = std::cos(angle_);
  float sz = std::sin(angle_);

  glm::mat4 const rz{
    { cz, -sz,  0,  0 },
    { sz,  cz,  0,  0 },
    { 0,    0,  1,  0 },
    { 0,    0,  0,  1 }};

  joint_angle_transform_ = glm::transpose(rz);
  local_transform_ = local_translation_ * local_rotation_ * joint_angle_transform_;
}

//
// Forward transform
// Parent * Translation * Rotation * Angle
//

const glm::mat4&
Joint::get_local_transform() const
{
  return local_transform_;
}

bool
Joint::has_degree_of_freedom() const
{
  if ((joint_type_ == JointType::continuous) or
      (joint_type_ == JointType::revolute)) {
    return true;
  }
  return false;
}
