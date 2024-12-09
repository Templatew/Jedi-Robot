// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      joint.h
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <utility>

#include <glm/vec3.hpp> // vec3
#include <glm/mat4x4.hpp> // mat4
                          //
enum class JointType: int {
    revolute,
    continuous,
    prismatic,
    fixed
};
extern std::ostream& operator<<(std::ostream& os, const JointType& jt);

class Link;

class Joint {
 private:
  JointType joint_type_;
  std::string name_;
  size_t idx_nonfixed_;  // idx of the nonfixed;
  Link* p_parent_link_;  // non-owning, can be nullptr
  Link* p_child_link_;   // non-owning, cannot be null
                         //
  float angle_;
  float min_angle_;
  float max_angle_;

  glm::vec3 local_xyz_;
  glm::vec3 local_rpy_;
  glm::mat4 local_translation_;
  glm::mat4 local_rotation_;
  glm::mat4 joint_angle_transform_;
  glm::mat4 local_transform_;

 public:
  void idx_nonfixed(size_t idx);
  size_t idx_nonfixed(void);

  Joint(const std::string& name);
  void info();
  bool has_degree_of_freedom() const;

  void add_parent_link(Link* plink) { p_parent_link_ = plink; }
  void add_child_link(Link* plink) { p_child_link_ = plink; }
  
  Link* parent_link() const { return p_parent_link_; }
  Link* child_link() const { return p_child_link_; }

  void set_joint_type(const std::string& joint_type, float min_angle = -1.57f, float max_angle = 1.57f );
  void set_local(const glm::vec3& t_xyz, const glm::vec3& r_rpy);
  //glm::mat4& set_transform(const glm::mat4& from_parent);
  //
  const glm::mat4& get_local_transform() const;

  void set_angle(const float angle);

  float get_angle() const
  {
    return angle_;
  }

  const std::string& name() const
  {
    return name_;
  }

  JointType get_joint_type() const 
  {
    return joint_type_;
  }

  std::pair<float, float> get_min_max_angles()
  {
    return std::make_pair(min_angle_, max_angle_);
  }

};
