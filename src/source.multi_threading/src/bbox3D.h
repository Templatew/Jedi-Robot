// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      bbox3D.h
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#pragma once

//#include <vector>
#include <glm/vec4.hpp> // vec3
#include <glm/mat4x4.hpp> // mat4
#include "glm_util.h"

/*
** outline of the boundary box structure
** it's a parallelepiped
**
**                           v4 ---- v7
**                          /|      /|
**                         / |     / |
**                        v5 ---- v6 |
**           z            |  |    |  |
**           |            |  v0 --|- v3
**           |            | /     | /
**           o-----y      |/      |/
**          /             v1 ---- v2
**         /
**        x
**
**
**        v0 v1 v2 v3 v4 v5 v6 v7
**  xmin  o  |  |  o  o  |  |  o
**  xmax  |  o  o  |  |  o  o  |
**  ymin  o  o  |  |  o  o  |  |
**  ymax  |  |  o  o  |  |  o  o
**  zmin  o  o  o  o  |  |  |  |
**  zmax  |  |  |  |  o  o  o  o
*/

class Bbox3d {
 private:
  std::vector<glm::vec4> vertices_;
  
 public:
  Bbox3d(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
  {
    glm::vec4 v0(xmin, ymin, zmin, 1.0f);
    glm::vec4 v1(xmax, ymin, zmin, 1.0f);
    glm::vec4 v2(xmax, ymax, zmin, 1.0f);
    glm::vec4 v3(xmin, ymax, zmin, 1.0f);

    glm::vec4 v4(xmin, ymin, zmax, 1.0f);
    glm::vec4 v5(xmax, ymin, zmax, 1.0f);
    glm::vec4 v6(xmax, ymax, zmax, 1.0f);
    glm::vec4 v7(xmin, ymax, zmax, 1.0f);

    vertices_.push_back(v0);
    vertices_.push_back(v1);
    vertices_.push_back(v2);
    vertices_.push_back(v3);

    vertices_.push_back(v4);
    vertices_.push_back(v5);
    vertices_.push_back(v6);
    vertices_.push_back(v7);

  }

  Bbox3d(): Bbox3d(0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f) {
  }


  void swap(Bbox3d& other)
  {
    for(size_t idx = 0; idx < vertices_.size(); ++idx) {
      glmutil::swap(vertices_[idx], other.vertices_[idx]);
    }
  }

  void clone_value_from(const Bbox3d& other)
  {
    for(size_t idx = 0; idx < vertices_.size(); ++idx) {
      vertices_[idx] = other.vertices_[idx];
    }
  }

  void transform_inplace(const glm::mat4& mat_transform)
  {
    for(size_t idx = 0; idx < vertices_.size(); ++idx) {
      vertices_[idx] = mat_transform * vertices_[idx];
    }
  }

  void transform_from(const Bbox3d& ref_bbox, const glm::mat4& mat_transform)
  {
    for(size_t idx = 0; idx < vertices_.size(); ++idx) {
      vertices_[idx] = mat_transform * ref_bbox.vertices_[idx];
    }
  }

  // distance between current and given bbox
  float distance(const Bbox3d& ref_bbox)
  {
    float sum = 0.0f;
    for(size_t idx = 0; idx < vertices_.size(); ++idx) {
      sum += glm::length(vertices_[idx] - ref_bbox.vertices_[idx]);
    }
    return sum;
  }

  std::ostream& print(std::ostream& os) const
  {
    auto& v0 = vertices_[0];
    auto& v1 = vertices_[1];
    auto& v2 = vertices_[2];
    auto& v4 = vertices_[4];

    os << "Bbox (xmin, xmax) = (" << v0.x << ", " << v1.x << ")\n";
    os << "Bbox (ymin, ymax) = (" << v0.y << ", " << v2.y << ")\n";
    os << "Bbox (zmin, zmax) = (" << v0.z << ", " << v4.z << ")\n";
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const Bbox3d& bbox)
  {
    return bbox.print(os);
  }


  bool intersects(const Bbox3d& other) const {
    return false;
  }

  bool below_ground() const
  {
    for(size_t idx = 0; idx < vertices_.size(); ++idx) {
      if (vertices_[idx].z < 0.0f) {
        return true;
      }
    }
    return false;
  }

};
