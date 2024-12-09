// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      link.h
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//
#pragma once

#include <fstream>
#include <functional>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <string>
#include <vector>

#include "bbox3D.h"

class Joint;

class Link {
 private:
  std::string name_;
  std::vector<Joint*> pjoints_;                  // vector of child joints pointer
  Link* p_parent_link_;                          // back pointer to parent (can be null)
  std::string mesh_name_;
  glm::mat4 global_transform_;
  Bbox3d bbox_original_;
  Bbox3d bbox_transformed_;
  bool is_target_;
  bool is_root_;

 public:
  Link(const std::string& name, const std::string& mesh_name);
  bool is_root() const;
  bool is_leaf() const;
  void info() const;
  void is_target(bool value) { is_target_ = value; }
  bool is_target() const { return is_target_; }

  void foreach_child_joint(std::function<void(Joint*)> fn);


  Link *parent_link() const { return p_parent_link_; }

  void add_parent_link(Link *plink) { p_parent_link_ = plink; }
  void add_child_joint(Joint *pjoint) { pjoints_.push_back(pjoint); }

  void set_bbox(const Bbox3d& bbox) { bbox_original_.clone_value_from(bbox); }

  const Bbox3d& get_bbox() const { return bbox_original_; }
  const Bbox3d& get_bbox_transformed() const { return bbox_transformed_; }


  const std::string& name() const { return name_; }

  void set_transform(const glm::mat4& global_transform) {
    global_transform_ = global_transform;
    bbox_transformed_.transform_from(bbox_original_, global_transform);
  }

  const glm::mat4& get_transform() const { return global_transform_; }
};
