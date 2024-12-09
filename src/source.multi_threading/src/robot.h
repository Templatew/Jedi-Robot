// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      robot.h
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#pragma once


#include <tuple>
#include <string>
#include <vector>
#include <random>
#include <set>
#include <random>


#include <glm/mat4x4.hpp> // mat4
#include <toml.hpp>

#include "ikparams.h"
#include "joint.h"
#include "link.h"
#include "poses.h"
#include "stlmesh.h" // Include the header file where StlMesh is defined


using ParentJointChild = std::tuple<std::string, std::string, std::string>;

class Robot {

 private:
  std::string name_;
  std::vector<Link> links_;
  std::vector<Joint > joints_;

  Joint *p_root_joint_;
  Link *p_root_link_;
  Link *p_target_link_;
  size_t degrees_of_freedom_;

  std::vector<size_t> nonfixed_joint_indexes_;

  glm::mat4 root_transform_;
  glm::mat4 target_transform_;
  std::set<ParentJointChild> joint_link_chains_;

  Joint& find_joint_from_name(const std::string& joint_name);
  Link& find_link_from_name(const std::string& link_name);

  void find_root_link();
  void validate();
  void create_links(toml::value& top_level_data);
  void create_joints(toml::value& top_level_data);
  void load_links();
  void load_joints();
  void add_link(const Link& link);
  void add_joint(Joint& ajoint, const std::string& parent_name, const std::string& child_name);

  std::vector<float> saved_angles_;
  std::vector<glm::mat4> saved_transforms_;
  float cyclic_coordinate_descent_single_joint(Joint& joint, float cyclic_coordinate_descent_cost);

  void propagate_transform(Joint &joint);

  Joint& get_ith_nonfixed_joint(size_t idx);

  bool check_collisions();

 public:

  size_t degrees_of_freedom() const {
    return degrees_of_freedom_;
  }

  Robot(const std::string& name, toml::value& top_level_data);
  const std::string& name() const { return name_; }
  void info() const;
  void save_config();
  void restore_config();

  bool forward_kinematic(const std::vector<float>& angles, bool verbose = false);
  float inverse_kinematic(IKParams& ik_params, float threshold, int angle_res_bits);

  //get joints
  const std::vector<Joint>& get_joints() const { return joints_; }

  void generate_blender_script(const std::string& filename) const;
  Pose get_current_pose() const;
  void init_from_pose(const Pose& pose);

  static std::vector<StlMesh> stl_meshes;
  static std::vector<Bbox3d> bboxes;
  static std::vector<std::string> link_names;
  static std::vector<std::string> stl_file_names;
  static std::vector<bool> is_targets; 

  static std::vector<std::string> joint_names;
  static std::vector<glm::vec3> translations3d;
  static std::vector<glm::vec3> rotations3d;
  static std::vector<std::string> joint_types;
  static std::vector<float> angles_min;
  static std::vector<float> angles_max;
  static std::vector<std::string> parent_names;
  static std::vector<std::string> child_names;
  
  static int robot_count;
};
