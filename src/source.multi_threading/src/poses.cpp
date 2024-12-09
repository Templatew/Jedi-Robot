// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      poses.cpp
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//


#include <chrono>
#include <fstream>
#include <vector>
#include <glm/mat4x4.hpp>
#include <glm/trigonometric.hpp>
#include <fmt/format.h>
#include <fmt/chrono.h>

#include "poses.h"
#include "context.h" 
#include "glm_util.h"

static void
write_blender_script_aux(std::ofstream& ostrm, const std::string& name,const glm::mat4& transform)
{
  ostrm << fmt::format("ob = bpy.context.scene.objects[\"{}\"]\n", name);  // # Get the object
  ostrm << fmt::format("bpy.ops.object.select_all(action='DESELECT')\n");  // # Deselect all objects
  ostrm << fmt::format("bpy.context.view_layer.objects.active = ob\n");    // # Make 'ob' the active object
  ostrm << fmt::format("ob.select_set(True)\n");

  std::vector<float> xyz_rpy = glmutil::get_xyz_rpy(transform);

  ostrm << fmt::format("bpy.context.object.location[0] = {}\n", xyz_rpy[0]);
  ostrm << fmt::format("bpy.context.object.location[1] = {}\n", xyz_rpy[1]);
  ostrm << fmt::format("bpy.context.object.location[2] = {}\n", xyz_rpy[2]);

  ostrm << fmt::format("bpy.context.object.rotation_euler[0] = {} #  {}\n", xyz_rpy[3], glm::degrees(xyz_rpy[3]));
  ostrm << fmt::format("bpy.context.object.rotation_euler[1] = {} #  {}\n", xyz_rpy[4], glm::degrees(xyz_rpy[4]));
  ostrm << fmt::format("bpy.context.object.rotation_euler[2] = {} #  {}\n", xyz_rpy[5], glm::degrees(xyz_rpy[5]));

  ostrm << fmt::format("ob.select_set(False)\n");
}


void
Pose::write_blender_script(std::ofstream& ostrm) const
{
  for(const auto& [name, transform, is_root, is_target] : link_transforms_) {
    write_blender_script_aux(ostrm, name, transform);
  }
}

void
Pose::write_blender_animation_script(std::ofstream& ostrm, int framenumber) const
{
  for(const auto& [name, transform, is_root, is_target] : link_transforms_) {
      ostrm << fmt::format("obj = bpy.data.objects['{}']\n", name);
      ostrm << fmt::format("obj.keyframe_insert(data_path='location',frame={})\n", framenumber);
      ostrm << fmt::format("obj.keyframe_insert(data_path='rotation_euler',frame={})\n", framenumber);
  }
}

void
Pose::write_blender_select_target(std::ofstream& ostrm) const
{
  for(const auto& [name, transform, is_root, is_target] : link_transforms_) {
    if (not is_target) {
      continue;
    }
    ostrm << fmt::format("ob = bpy.context.scene.objects['{}']\n", name);
    ostrm << fmt::format("bpy.ops.object.select_all(action='DESELECT')\n");
    ostrm << fmt::format("bpy.context.view_layer.objects.active = ob\n");
    ostrm << fmt::format("ob.select_set(True)\n");
    break;
  }
}

float
Pose::get_joint_angle(const std::string& name) const
{
  for(auto&  [ith_name, ith_angle] : joint_angles_) {
    if (ith_name == name) {
      return ith_angle;
    }
  }
  throw robot::exception("Cannot find joint with name '{}'", name);
}

void
Pose::add_link_transform(const std::string& name, const glm::mat4& transform, bool is_root, bool is_target)
{
  link_transforms_.emplace_back(name, transform, is_root, is_target);
}

void
Pose::add_joint_angle(const std::string& name, float angle)
{
  joint_angles_.emplace_back(name, angle);
}


const glm::mat4& 
Pose::get_link_transform(const std::string& name) const
{
  for(const auto& [ith_name, ith_transform, is_root, is_target] : link_transforms_) {
    if (ith_name == name) {
      return ith_transform;
    }
  }
  throw robot::exception("Cannot find link with name '{}'", name);
}

///////////////////////////////////
//
// Poses
//
///////////////////////////////////

void
Poses::write_blender_script(const std::string& filename) const
{
  logger::info("Generate blender script to file {}", filename);

  try {
    std::ofstream ostrm(filename);
    ostrm.exceptions(std::ofstream::eofbit | std::ofstream::failbit | std::ofstream::badbit);


    //auto t = std::time(nullptr);
    //auto tm = *std::gmtime(&t);
    //ostrm << fmt::format(fmt::runtime("# file generated on {:%Y-%m-%d %H:%M:%S}\n"), tm);
    
    auto utc_now = std::chrono::system_clock::now();
    auto utc_time = std::chrono::system_clock::to_time_t(utc_now);

    ostrm << fmt::format("# blender file {}\n", filename);
    ostrm << fmt::format("# file generated on {:%F %T %Z}\n", *std::gmtime(&utc_time));
    ostrm << fmt::format("# by {}\n", RunContext::instance().get_username());
    ostrm << fmt::format("#\n");
    ostrm << fmt::format("# to load this file, on the blender python console type the following\n");
    ostrm << fmt::format("# for linux:\n");
    ostrm << fmt::format("# myfile = '/the/full/path/to/{}'\n", filename);
    ostrm << fmt::format("# for windows\n");
    ostrm << fmt::format("# myfile = 'C:\\\\users\\\\username\\\\somefolder\\\\{}'\n", filename);
    ostrm << fmt::format("# then type:\n");
    ostrm << fmt::format("# exec(compile(open(myfile).read(), myfile, 'exec'))\n");
    ostrm << fmt::format("#\n");
    ostrm << fmt::format("# blender commands below\n");

    if (poses_.size() == 0) {
      return;
    }
    
    int framedelta = 1; 
    int framenumber = 0;
    int idx = 0;
    for(const auto& pose: poses_) { 
      ostrm << fmt::format("#\n");
      ostrm << fmt::format("# link transform for pose index {}\n", idx);
      ostrm << fmt::format("#\n");
      pose.write_blender_script(ostrm);
      pose.write_blender_animation_script(ostrm, framenumber);
      ++idx;
      framenumber += framedelta;
    }

    ostrm << fmt::format("#\n");
    ostrm << fmt::format("# select the target object\n");
    ostrm << fmt::format("#\n");
    poses_[0].write_blender_select_target(ostrm);

    ostrm << fmt::format("#\n");
    ostrm << fmt::format("# end of blender file\n");
    ostrm << fmt::format("#\n");
    ostrm << std::endl;

    logger::info("Script file generated with {} poses", idx);

  } catch (const std::ios_base::failure& error) {
    std::cerr << "INFO: catch i/o failure " << error.what();
    std::abort();
  }

  catch (const std::bad_alloc& error) {
    std::cerr << "INFO: catch bad_alloc " << error.what();
    std::abort();
  }

  catch (const std::exception& error) {
    std::cerr << "INFO: catch an error " << error.what();
    std::abort();
  }
}




#if 0
    // use default position
    for (const auto& link : links_) {
      link.print_blender_script(ostrm, true);
    }

    for (const auto& link : links_) {
      const std::string& name = link.name();
      ostrm << "obj = bpy.data.objects['" << name << "']\n";
      ostrm << "obj.keyframe_insert(data_path='location',frame=1)\n";
      ostrm << "obj.keyframe_insert(data_path='rotation_euler',frame=1)\n";
    }

    ostrm << "#\n";
    ostrm << "# set final position\n";
    ostrm << "#\n";

    for (const auto& link : links_) {
      link.print_blender_script(ostrm);
    }

    int last_frame = 180;

    ostrm << "#\n";
    ostrm << "# assign final position to keyframe " << last_frame << "\n";
    ostrm << "#\n";

    for (const auto& link : links_) {
      const std::string& name = link.name();
      ostrm << "obj = bpy.data.objects['" << name << "']\n";
      ostrm << "obj.keyframe_insert(data_path='location',frame=" << last_frame
            << ")\n";
      ostrm << "obj.keyframe_insert(data_path='rotation_euler',frame="
            << last_frame << ")\n";
    }

    // select the target object

    ostrm << "#\n";
    ostrm << "# select the target object\n";
    ostrm << "#\n";
    for (const auto& link : links_) {
      if (link.is_target()) {
        ostrm << "ob = bpy.context.scene.objects[\"" << link.name() << "\"]\n";
        ostrm << "bpy.ops.object.select_all(action='DESELECT')\n";
        ostrm << "bpy.context.view_layer.objects.active = ob\n";
        ostrm << "ob.select_set(True)\n";
        break;
      }
    }
#endif

