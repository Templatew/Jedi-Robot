// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      glm_util.h
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#pragma once

#include <glm/mat4x4.hpp>  // mat4
#include <glm/vec3.hpp>    // vec3
#include <glm/vec4.hpp>    // vec3
#include <iostream>
#include <vector>
 
extern std::ostream& operator<<(std::ostream& os, const glm::vec3& vs);
extern std::ostream& operator<<(std::ostream& os, const glm::vec4& vs);
extern std::ostream& operator<<(std::ostream& os, const glm::mat4& vs);


namespace glmutil {

glm::mat4 to_matrix4x4(const std::vector<float> xyx, const std::vector<float> rpy);
extern bool close_enough(float value, float target);
extern float normalize_angle(float);
extern void swap(glm::vec4& lhs, glm::vec4& rhs);
extern glm::vec3 get_position(const glm::mat4& m1);
extern std::vector<float> get_xyz_rpy(const glm::mat4& m);
}  // namespace glmutil
