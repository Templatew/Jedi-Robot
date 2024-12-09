// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      params.h
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <random>

#include <glm/mat4x4.hpp>  // mat4
#include <glm/vec3.hpp>    // vec3
#include <glm/vec4.hpp>    // vec3


struct IKParams
{
  std::vector<float> xyz_;
  std::vector<float> rpy_;
  uint32_t seed_;
  size_t nb_iterations_;
  std::mt19937 rnd_generator_;
  std::string method_;            // store the IK method variant 

  IKParams(
      const std::vector<float>& xyz, 
      const std::vector<float>& rpy, 
      uint32_t seed, 
      size_t nb_iteration,
      const std::string& method) :
    xyz_{xyz}, 
    rpy_{rpy},
    seed_{seed},
    nb_iterations_{nb_iteration},
    method_{method}
  {
    rnd_generator_.seed(seed_);

  }
};


