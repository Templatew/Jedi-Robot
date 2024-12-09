// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      stlmesh.h
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <exception>
#include <functional>
#include <tuple>
#include <memory>
#include <glm/mat4x4.hpp> // mat4
#include <glm/vec3.hpp> // vec3


#include "facet.h"
#include "bbox3D.h"

using ThreeFloats = std::tuple<float, float, float>;
std::ostream &operator<<(std::ostream& ostrm, const ThreeFloats& v);

// for the exception

struct stlmesh
{

  class exception : public std::exception {
   private:
    std::string msg_;
   public:
    explicit exception(std::string const& msg) : msg_{msg} {}
    const char* what() const noexcept override {
      return msg_.c_str();
    }
  };
};

// Array of facets
// Polygonal face element (see below)
// from 'f' in OBJ file, either
// f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3 ...
// f 1 2 3   => vertex only
// f 3/1 4/2 5/3  => vertex/texture
// f 6/4/1 3/5/3 7/6/5 => vertex/texture/normal
//


class StlMesh
{
 private:
  inline static size_t cnt_ = 0;

  std::vector<Facet> facets_;
  std::vector<glm::vec4> facets_normals_;
  std::vector<glm::vec3> facets_raw_normals_;
  std::vector<glm::vec4> vertices_;
  std::vector<glm::vec3> vertex_normals_;

  std::string name_;
  size_t id_;
  uint64_t load_time_in_ms_;

  Bbox3d bbox_;

 public:
  StlMesh(std::string const &name): name_{name} {
    id_ = cnt_;
    ++cnt_;
  }

  void load_time(uint64_t t_in_ms)
  {
    load_time_in_ms_ = t_in_ms;
  }

  uint64_t load_time() const
  {
    return(load_time_in_ms_);
  }

  size_t get_id() const {
    return id_;
  }

  std::string &get_name() {
    return name_;
  }

  ~StlMesh() = default;


  void add_vertex(const glm::vec4 &v4d);
  void add_vertex(float x, float y, float z);
  void add_texture_uv(float u, float v);

  void add_facet(size_t idx0, size_t idx1, size_t idx2) {
    facets_.emplace_back(idx0, idx1, idx2);
  }

  Facet &get_facet(size_t idx) {
    return facets_[idx];
  }

  void add_facet(size_t idx0, size_t idx1, size_t idx2, size_t t0, size_t t1, size_t t2) {
    facets_.emplace_back(idx0, idx1, idx2, t0, t1, t2);
  }

  void compute_bbox();

  Bbox3d& get_bbox()
  {
    return bbox_;
  }

  bool check();
  void info();

  size_t nb_vertex() {
    return vertices_.size();
  }
};


class StlMeshFactory
{
private:
  //  using PairOfIdx = std::pair<size_t, size_t>;
  struct HashV3D {
    size_t operator()(const ThreeFloats& akey) const noexcept {
      size_t hash0 = std::hash<float>{}(std::get<0>(akey));
      size_t hash1 = std::hash<float>{}(std::get<1>(akey));
      size_t hash = hash1 ^ (hash0 << 1);
      size_t hash2 = std::hash<float>{}(std::get<2>(akey));
      hash = hash2 ^ (hash << 1);
      return hash;
    }
  };
  struct KeyCompV3D {
    bool operator()(const ThreeFloats& a, const ThreeFloats& b) const noexcept {
      bool res = (std::get<0>(a) == std::get<0>(b)) and
                 (std::get<1>(a) == std::get<1>(b)) and
                 (std::get<2>(a) == std::get<2>(b));
      return res;
    }
  };
 private:
  StlMeshFactory();
  void add_mesh(const std::string&, StlMesh& );

  using Name2MeshDict = std::unordered_map<std::string, StlMesh>;

  Name2MeshDict name2mesh_;

 public:
  static StlMeshFactory& instance() {
    static StlMeshFactory factory;
    return factory;
  }

  StlMesh& get_mesh(const std::string&);

  StlMesh& create_from_stlfile(const std::string& name, const std::string& file_name, const float scale_factor = 1.0f);

};


