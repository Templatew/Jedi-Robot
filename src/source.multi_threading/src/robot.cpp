// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      robot.cpp
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#include <algorithm>
#include <random>
//#include <glm/glm.hpp>                // mat4
#include <glm/trigonometric.hpp>
#include <glm/mat4x4.hpp>
#include <map>
#include <memory>
#include <string>
#include <toml.hpp>
#include <vector>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include "context.h"
#include "joint.h"
#include "link.h"
#include "logger.h"
#include "stlmesh.h"
#include "robot.h"
#include "poses.h"

// Define static member variables

int Robot::robot_count = 0;

std::vector<std::string> Robot::link_names;

std::vector<std::string> Robot::stl_file_names;

std::vector<Bbox3d> Robot::bboxes;

std::vector<StlMesh> Robot::stl_meshes;

std::vector<bool> Robot::is_targets;

std::vector<std::string> Robot::joint_names;

std::vector<glm::vec3> Robot::translations3d;

std::vector<glm::vec3> Robot::rotations3d;

std::vector<std::string> Robot::joint_types;

std::vector<float> Robot::angles_min;

std::vector<float> Robot::angles_max;

std::vector<std::string> Robot::parent_names;

std::vector<std::string> Robot::child_names;


// needed to parse a number in the toml file into a float
// example angle_min = 1.57



static float
get_float(const std::string& joint_name, toml::table& a_table, const std::string& a_key)
{
  toml::value& an_item = a_table.at(a_key);
  float v = 0.0f;
  if (an_item.type() == toml::value_t::floating) {
    v = static_cast<float>(an_item.as_floating());
  } else if (an_item.type() == toml::value_t::integer) {
    v = static_cast<float>(an_item.as_integer());
  } else {
    throw robot::exception("Entry of key '{}' is not a value for joint '{}'",
                           a_key, joint_name);
  }
  return v;
}

// parse a vector of three floats
// example rpy = [-3.141594, 0, 0]

static
glm::vec3 get_vec3(const std::string& joint_name, toml::table& a_table, const std::string& a_key)
{
  glm::vec3 res3d;
  toml::array an_array = toml::get<toml::array>(a_table.at(a_key));

  for (size_t idx = 0; idx < 3; ++idx) {
    toml::value& an_item = an_array.at(idx);
    float v = 0.0f;
    if (an_item.type() == toml::value_t::floating) {
      v = static_cast<float>(an_item.as_floating());
    } else if (an_item.type() == toml::value_t::integer) {
      v = static_cast<float>(an_item.as_integer());
    } else {
      throw robot::exception(
          "Entry of vector '{}' is not a value for joint '{}'", a_key,
          joint_name);
    }
    res3d[idx] = v;
  }
  return res3d;
}

//
// create a robot based on a config file
//

Robot::Robot(const std::string& name, toml::value& top_level_data)
    : name_{name},
      root_transform_{glm::mat4(1.0f)},
      target_transform_{glm::mat4(1.0f)}
{ 
  if (robot_count==0){
    create_links(top_level_data);
    create_joints(top_level_data);
  }
  else {
    load_links();
    load_joints();
  }

  validate();
  robot_count++;

  // compute the degrees of freedom and 
  // quick access to non fixed joints
  
  degrees_of_freedom_ = 0;
  for (size_t idx = 0; idx < joints_.size(); ++idx) {
    if (joints_[idx].has_degree_of_freedom()) {
      ++degrees_of_freedom_;
      joints_[idx].idx_nonfixed(nonfixed_joint_indexes_.size());
      nonfixed_joint_indexes_.push_back(idx);
    }
  }
}

void Robot::load_links(){

  for (std::vector<std::string>::size_type i = 0; i < link_names.size(); i++){
    Link link(link_names[i] , stl_file_names[i]);
    logger::debug("Loading link: {}", link_names[i]);
    link.set_bbox(bboxes[i]);
    link.is_target(is_targets[i]);
    add_link(link);
  }
}

void Robot::load_joints(){

  for (std::vector<std::string>::size_type i = 0; i < joint_names.size(); i++){
    Joint joint(joint_names[i]);
    joint.set_local(translations3d[i], rotations3d[i]);
    joint.set_joint_type(joint_types[i], angles_min[i], angles_max[i]);
    add_joint(joint, parent_names[i], child_names[i]);
  }
}


// [[links]]
//   name = 'L00.Base'
//   path = 'meshes/base_link.stl'
//   istarget = 1   (optional)
//   isroot   = 1   (optional)

void
Robot::create_links(toml::value& top_level_data)
{
  // list of mandatory entries
  std::map<std::string, decltype(toml::value_t::table)> link_entries{
      {"name", toml::value_t::string},
      {"path", toml::value_t::string}
  };

  try {
    auto& tbl_info = toml::find(top_level_data, "info");
    const auto stl_root_path = toml::find<std::string>(tbl_info, "root_path");
    auto list_of_links =
        toml::find<std::vector<toml::table>>(top_level_data, "links");
    auto& mesh_factory = StlMeshFactory::instance();

    // loop over all the entries in [[links]]
    //
    for (toml::table& a_table : list_of_links) {

      // verify that the table has all the entries
      for (const auto& an_entry : link_entries) {
        auto& [a_key, a_type] = an_entry;
        if (a_table.count(a_key) == 0) {
          throw robot::exception("Missing '{0}' entry in table 'links'", a_key);
        }
        if (a_table.at(a_key).type() != a_type) {
          throw robot::exception("Entry '{}' has wrong type in table 'links'", a_key);
        }
      }

      // read the associated mesh and compute the bbox

      std::string& link_name = a_table["name"].as_string();
      link_names.push_back(link_name);
      std::string& stl_file_name = a_table["path"].as_string();
      stl_file_names.push_back(stl_file_name);
      Link link(link_name, stl_file_name);

      std::string stl_filepath = stl_root_path + "/" + stl_file_name;
      auto& mesh = mesh_factory.create_from_stlfile(link_name, stl_filepath);
      auto& bbox = mesh.get_bbox();
      bboxes.push_back(bbox);
      stl_meshes.push_back(mesh);
      link.set_bbox(bbox);
      // take care of optional configuration
      bool is_target = false;
      if (a_table.count("istarget") != 0) {
        is_target = true;
      }
      link.is_target(is_target);
      is_targets.push_back(is_target);
      add_link(link);
      mesh.info();
    }
  } catch (const robot::exception& err) {
    logger::fatal(err.what());
    std::exit(1);
  }
}

// [[joints]]
//   name = 'J00'
//   type = 'continous'
//   parent = 'L00.Base'
//   child = 'L01.Shoulder'
//   xyz = [ 0, 0, 0.15643 ]
//   rpy = [-3.141594, 0, 0]
//   axis = [0, 0, 1]
//   angle_min = 0
//   angle_max = 0
//   angle_default = 0
void
Robot::create_joints(toml::value& top_level_data)
{

  // list mandatory entries for each joint
  std::map<std::string, decltype(toml::value_t::table)> joint_entries{
      {"name", toml::value_t::string},
      {"type", toml::value_t::string},
      {"parent", toml::value_t::string},
      {"child", toml::value_t::string},
      {"xyz", toml::value_t::array},
      {"rpy", toml::value_t::array},
      {"axis", toml::value_t::array},
      {"angle_min", toml::value_t::floating},
      {"angle_max", toml::value_t::floating},
      {"angle_default", toml::value_t::floating}};

  //
  // create the joints
  //
  try {
    auto list_of_joints =
        toml::find<std::vector<toml::table>>(top_level_data, "joints");

    // iterates over all the  [[joints]]
    //
    for (toml::table& a_table : list_of_joints) {
      // verify that the table has all the entries
      for (const auto& an_entry : joint_entries) {
        auto& [a_key, a_type] = an_entry;
        if (a_table.count(a_key) == 0) {
          throw robot::exception("Missing '{0}' entry in table 'joints'", a_key);
        }
        if (a_type == toml::value_t::floating and
            a_table.at(a_key).type() != toml::value_t::floating and
            a_table.at(a_key).type() != toml::value_t::integer) {
          throw robot::exception("Entry '{}' is not a value in table 'joints'", a_key);

        } else if (a_type != toml::value_t::floating and
                   a_table.at(a_key).type() != a_type) {
          throw robot::exception("Entry '{}' has wrong type in table 'joints'", a_key);
        }
      }

      std::string& joint_name = a_table["name"].as_string();
      joint_names.push_back(joint_name);
      Joint ajoint(joint_name);
      glm::vec3 translation3d = get_vec3(joint_name, a_table, "xyz");
      translations3d.push_back(translation3d);
      glm::vec3 rpy3d = get_vec3(joint_name, a_table, "rpy");
      rotations3d.push_back(rpy3d);
      // std::cout << "[DEBUG] processing joint  " << joint_name << std::endl;
      // std::cout << "[DEBUG] joint translation " << translation3d <<
      // std::endl; std::cout << "[DEBUG] joint rotation    " << rpy3d <<
      // std::endl;
      ajoint.set_local(translation3d, rpy3d);

      std::string& joint_type = a_table["type"].as_string();
      joint_types.push_back(joint_type);
      float angle_min = get_float(joint_name, a_table, "angle_min");
      angles_min.push_back(angle_min);
      float angle_max = get_float(joint_name, a_table, "angle_max");
      angles_max.push_back(angle_max);

      ajoint.set_joint_type(joint_type, angle_min, angle_max);

      std::string& parent_name = a_table["parent"].as_string();
      parent_names.push_back(parent_name);
      std::string& child_name = a_table["child"].as_string();
      child_names.push_back(child_name);

      add_joint(ajoint, parent_name, child_name);
    }
  } catch (const robot::exception& err) {
    logger::fatal(err.what());
    std::exit(1);
  }
}


//
//   <<<<LINK>>> <<JOINT>> <<<<LINK>>>>
//
Link&
Robot::find_link_from_name(const std::string& link_name)
{
  for (auto& a_link : links_) {
    if (a_link.name() == link_name) {
      return a_link;
    }
  }
  throw robot::exception("Cannot find link with name '{}'", link_name);
}

Joint&
Robot::find_joint_from_name(const std::string& joint_name)
{
  for (auto& a_joint : joints_) {
    if (a_joint.name() == joint_name) {
      return a_joint;
    }
  }
  throw robot::exception("Cannot find joint with name '{}'", joint_name);
}


void
Robot::add_joint(Joint& a_joint, const std::string& parent_name, const std::string& child_name)
{
  const std::string& joint_name = a_joint.name();

  // validate that the joint name is unique
  // we could have used a map, but number of ajoint is small
  // so we just iterate of each joints

  bool named_is_used = std::ranges::any_of( joints_,
      [&joint_name](const Joint& ith_joint) -> bool {
        return ith_joint.name() == joint_name;
      });

  if (named_is_used) {
    throw robot::exception("Joint name '{}' already used, aborting.", joint_name);
  }

  // validate the parent and child link
  // have already been created

  [[maybe_unused]] Link& parent_link = find_link_from_name(parent_name);
  [[maybe_unused]] Link& child_link = find_link_from_name(child_name);

  // validation of the correctness of the joint
  // * cannot have a joint on an identical existing joint
  // * cannot have a joint creating a loop

  ParentJointChild pjc_item{parent_name, joint_name, child_name};
  if (joint_link_chains_.count(pjc_item) != 0) {
    throw robot::exception(
        "Already have a joint name '{}' between parent and child links",
        joint_name);
  }

  ParentJointChild pjc_item_inverse{child_name, joint_name, parent_name};
  if (joint_link_chains_.count(pjc_item_inverse) != 0) {
    throw robot::exception(
        "Already have a joint name '{}' between parent and child links",
        joint_name);
  }

  // validation of the correctness of the joint
  // *  cannot have a joint with re-convergent parents
  //    if parent1->jointX->child3 exits we cannot have
  //    parent2->jointX->child3
  //
  //     Link.parent1
  //                |
  //                jointX--Link.child3
  //                |
  //     Link.parent2
  //
  for (auto& a_pjc_item : joint_link_chains_) {
    auto& child_name_used = std::get<2>(a_pjc_item);
    if (child_name_used == child_name) {
      throw robot::exception(
          "Joint  name '{}', child link '{}' is already used, aborting",
          joint_name, child_name);
    }

    auto& parent_name_used = std::get<0>(a_pjc_item);
    if (child_name_used == parent_name and parent_name_used == child_name) {
      throw robot::exception(
          "Joint  name '{}', parent link '{}' is already used, aborting",
          joint_name, child_name);
    }
  }
  joint_link_chains_.insert(pjc_item);
  joints_.push_back(a_joint);
}

  // warp-up here
  // connect the joint to the links.
  //
  // a_joint.set_link_names(parent_name, child_name);
  //
  // parent_link.add_joint_child_side(joint_name);
  // child_link.add_joint_parent_side(joint_name);


void
Robot::add_link(const Link& a_link)
{
  const std::string& link_name = a_link.name();

  // validate that the name is unique
  // we could have used a map, but number of links is small
  // so we just iterate of each links

  bool named_is_used = std::ranges::any_of(links_,
    [&link_name](const Link& ith_link) -> bool {
      return ith_link.name() == link_name;
    });

  if (named_is_used) {
    throw robot::exception("Link name '{}' already used, aborting", link_name);
  }

  links_.push_back(a_link);
}


//
// find_root_link()
// make sure that only 1 root link exists
// iterating over the joint_link_chains_
// { parent link, joint, child link }
// if a link appears only on position [0] then is a root joint

void
Robot::find_root_link()
{
  std::map<std::string, int> link_name_counter;
  for (auto& a_pjc : joint_link_chains_) {
    auto& parent_name = std::get<0>(a_pjc);
    if (link_name_counter.count(parent_name) == 0) {
      link_name_counter.insert(std::make_pair(parent_name, 0));
    }
    auto& child_name = std::get<2>(a_pjc);
    if (link_name_counter.count(child_name) == 0) {
      link_name_counter.insert(std::make_pair(child_name, 1));
    } else  {
      ++link_name_counter[child_name];
    }
  }
  bool found_it = false;
  std::string root_link_name = "";
  for (const auto& [link_name, counter] : link_name_counter) {
    if (counter == 0) {
      if (found_it) {
        throw robot::exception("Multiple root links found with name '{}' and '{}'", root_link_name, link_name);
      }
      root_link_name = link_name;
      found_it = true;
    }
  }
  if (not found_it) {
    throw robot::exception("Cannot find root link");
  }
  p_root_link_ = std::addressof(find_link_from_name(root_link_name));
 }


//
// various sanity check on the robots
// and adding pointers for chain linking the Links and the Joints
//

void
Robot::validate()
{
  // step 1.
  // find the root link

  find_root_link();

  // step 2.
  // creates pointer links

  for (auto& a_pjc : joint_link_chains_) {
    Link*  parent_link_ptr = std::addressof(find_link_from_name(  std::get<0>(a_pjc)));
    Joint* joint_ptr       = std::addressof(find_joint_from_name( std::get<1>(a_pjc)));
    Link*  child_link_ptr  = std::addressof(find_link_from_name(  std::get<2>(a_pjc)));

    parent_link_ptr->add_child_joint(joint_ptr);
    child_link_ptr->add_parent_link(parent_link_ptr);
    joint_ptr->add_parent_link(parent_link_ptr);
    joint_ptr->add_child_link(child_link_ptr);
  }

  // step 3.
  // set target link and verify only one target is set

  bool found_it = false;
  p_target_link_ = nullptr;
  for (auto& a_link : links_) {
    if (a_link.is_target()) {
      if (not found_it) {
        p_target_link_ = std::addressof(a_link);
        found_it = true;
      } else {
        throw robot::exception("Multiple target found with name '{}' and '{}'",
            p_target_link_->name(), a_link.name());
      }
    }
  }
  if (not found_it) {
    throw robot::exception("Could not find a target link");
  }
}


void
Robot::info() const
{
  logger::info("+++++++++++++++++++++++++++++++++++++++");
  logger::info("Information for robot '{}'", name_);
  logger::info("Number of links     {}", links_.size());
  logger::info("Number of joints    {}", joints_.size());
  logger::info("Degrees of freedom  {}", degrees_of_freedom());
  logger::info("Root link           {}", p_root_link_->name());
  logger::info("Target link         {}", p_target_link_->name());
  logger::info("+++++++++++++++++++++++++++++++++++++++");
}


void
Robot::propagate_transform(Joint &joint)
{
  Link* plink = joint.parent_link();

  std::vector<Link*> plinks;
  plinks.push_back(plink);

  do {
    plink = plinks[0];
    plinks.erase(plinks.begin());

    plink->foreach_child_joint([&plinks](Joint* pjoint) {
      glm::mat4 local_transform = pjoint->get_local_transform();
      Link* p_child_link = pjoint->child_link();
      Link* p_parent_link = p_child_link->parent_link();

      const glm::mat4& parent_transform = p_parent_link->get_transform();
      p_child_link->set_transform(parent_transform * local_transform);
      plinks.push_back(p_child_link);
    });
  } while (plinks.size() != 0);

}



Pose
Robot::get_current_pose() const
{
  Pose pose;
  for(const auto& link: links_) {
    pose.add_link_transform(link.name(), link.get_transform(), link.is_root(), link.is_target());
  }

  for(const auto& joint: joints_) {
    pose.add_joint_angle(joint.name(), joint.get_angle());
  }

  return pose;
}

void
Robot::init_from_pose(const Pose& pose)
{
  for(auto& link: links_) {
    const glm::mat4& transform = pose.get_link_transform(link.name());
    link.set_transform(transform);
  }

  for(auto& joint: joints_) {
    const float angle = pose.get_joint_angle(joint.name());
    joint.set_angle(angle);
  }

}


void
Robot::save_config()
{
  saved_angles_.clear();
  for(const auto& joint: joints_) {
    if (joint.has_degree_of_freedom()) {
      saved_angles_.push_back(joint.get_angle());
    }
  }

  saved_transforms_.clear();
  for(auto& link: links_) {
    const glm::mat4& transform = link.get_transform();
    saved_transforms_.push_back(transform);
  }
}

void
Robot::restore_config()
{
  size_t idx;

  idx = 0;
  for(auto& joint: joints_) {
    if (joint.has_degree_of_freedom()) {
      joint.set_angle(saved_angles_[idx]);
      ++idx;
    }
  }

  idx = 0;
  for(auto& link: links_) {
    link.set_transform(saved_transforms_[idx]);
    ++idx;
  }
}

Joint&
Robot::get_ith_nonfixed_joint(size_t idx)
{
  assert(idx < nonfixed_joint_indexes_.size());
  size_t idx_nonfixed = nonfixed_joint_indexes_[idx];

  assert(idx == joints_[idx_nonfixed].idx_nonfixed());
  return joints_[idx_nonfixed];
}

//
// forward kinematic
//

bool
Robot::forward_kinematic(const std::vector<float>& angles, bool verbose)
{
  std::vector<float> angles_in_degrees;
  for(const auto& value: angles) {
    angles_in_degrees.push_back(glm::degrees(value));
  }

  if(verbose){
    logger::info("Compute forward kinematic with angles {:g}", fmt::join(angles_in_degrees, ","));
  }
  
  glm::mat4 current_transform = root_transform_;

  for (auto& joint : joints_) {

      if (joint.has_degree_of_freedom()) {
          size_t idx = joint.idx_nonfixed();
          joint.set_angle(angles[idx]);
      }

      glm::mat4 local_transform = joint.get_local_transform();

      current_transform *= local_transform;
      propagate_transform(joint);
  }

  const glm::mat4& target_transform = p_target_link_->get_transform();
  std::vector<float> xyz_rpy = glmutil::get_xyz_rpy(target_transform);

  if (verbose){
    logger::info("Target position (x, y, z) = ({}, {}, {})", xyz_rpy[0], xyz_rpy[1] , xyz_rpy[2]);
    logger::info("Target rotation (r, p, y) = ({}, {}, {})",
        glm::degrees(xyz_rpy[3]), 
        glm::degrees(xyz_rpy[4]),
        glm::degrees(xyz_rpy[5]));
    std::cout << "Target Transform = "; std::cout << target_transform << std::endl;
  }
  return true ;
}

bool Robot::check_collisions(){

  for (auto &link: links_){
    if (link.get_bbox_transformed().below_ground()){return true;}
  }
  return false;
}

//
// inverse kinematic
// the param is not const to allow iteration of the rnd generator
//
float
Robot::inverse_kinematic(IKParams& ik_params, float threshold, int angle_res_bits)
{
  std::vector<float> rpy_in_degrees;
  for(const auto& value: ik_params.rpy_) {
    rpy_in_degrees.push_back(glm::degrees(value));
  }

  target_transform_ = glmutil::to_matrix4x4(ik_params.xyz_, ik_params.rpy_);
  float ccd_distance_to_target = std::numeric_limits<float>::max();
  
  std::mt19937 rng(ik_params.seed_);
  std::uniform_int_distribution<size_t> dist(0, degrees_of_freedom_ - 1);
  
  Bbox3d target_bbox = p_target_link_->get_bbox();
  target_bbox.transform_inplace(target_transform_);

  Bbox3d bbox = p_target_link_->get_bbox();
  Bbox3d new_bbox;
  ccd_distance_to_target = bbox.distance(target_bbox);

  float angle_res = 360.0f / (1 << angle_res_bits);


  for (size_t iter = 0; iter < ik_params.nb_iterations_; ++iter) {
  
    size_t joint_idx = dist(rng);
    Joint& joint = get_ith_nonfixed_joint(joint_idx);

    float best_angle = joint.get_angle();
        
    auto [min_angle, max_angle] = joint.get_min_max_angles();

    float deviation_up = std::max((max_angle - best_angle) / (iter + 1), angle_res);
    float deviation_down = std::max((best_angle - min_angle) / (iter + 1)*2, angle_res);

    for (float angle = std::max(best_angle - deviation_down, min_angle); angle <= std::min(best_angle + deviation_up, max_angle); angle += angle_res) {
      
      joint.set_angle(angle);
      propagate_transform(joint);
        
      new_bbox.transform_from(bbox, p_target_link_->get_transform());
      float new_distance = new_bbox.distance(target_bbox);

      if (new_distance < ccd_distance_to_target){

        if (ik_params.method_ == "collisions" && check_collisions()){
          continue;
        }

        if (new_distance < threshold){
          return new_distance;
        }

        ccd_distance_to_target = new_distance;
        best_angle = angle;


      }
    }

    joint.set_angle(best_angle);
    propagate_transform(joint);
  }

  return ccd_distance_to_target;
}