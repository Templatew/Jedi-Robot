// header-start
////////////////////////////////////////////////////////////////////////////////
//
// \file      stlmesh.cpp
// \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
//
////////////////////////////////////////////////////////////////////////////////
// header-end
//

#include "stlmesh.h"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unordered_map>

#include "bbox3D.h"
#include "logger.h"
#include "util.h"

std::ostream &operator<<(std::ostream &ostrm, const ThreeFloats &v) {
  ostrm << "(" << std::get<0>(v) << "," << std::get<1>(v) << ","
        << std::get<2>(v) << ")";
  return ostrm;
}

StlMeshFactory::StlMeshFactory() {}

#if 0
void
StlMesh::compute_vertex_normal()
{
  // step #1
  // initialize vertex normal to be all 0
  for(size_t idx = 0; idx < vertices_.size(); ++idx) {
    vertex_normals_.emplace_back(0.0f, 0.0f, 0.0f);
  }

  // step #2
  // for each of the facets
  // update the vertices linked to that facet
  for(size_t idx = 0; idx < facets_.size(); ++idx) {
    size_t idx0 = facets_[idx].get<0>();
    size_t idx1 = facets_[idx].get<1>();
    size_t idx2 = facets_[idx].get<2>();

    vertex_normals_[idx0] += facets_raw_normals_[idx];
    vertex_normals_[idx1] += facets_raw_normals_[idx];
    vertex_normals_[idx2] += facets_raw_normals_[idx];
  }

  // step #
  // normalize the result vertex normal to be all 0
  for(size_t idx = 0; idx < vertices_.size(); ++idx) {
    vertex_normals_[idx] = glm::normalize(vertex_normals_[idx]);
  }
}


StlMesh::compute_facet_normal()
{
  using namespace std::numbers;

  for(auto &facet: facets_) {
    size_t idx0 = facet.get<0>();
    size_t idx1 = facet.get<1>();
    size_t idx2 = facet.get<2>();

    glm::vec3 p0 = vertices_[idx0].toVector3D();
    glm::vec3 p1 = vertices_[idx1].toVector3D();
    glm::vec3 p2 = vertices_[idx2].toVector3D();

    // as facet normal is a vector
    // the normalize 4th value is set to 0.
    glm::vec3 vnormal = glm::vec3::normal(p0, p1, p2);
    facets_normals_.emplace_back(glm::vec4{vnormal[0], vnormal[1], vnormal[2], 0.0f});

    glm::vec3 cross_product = glm::cross(p1 - p0, p2 - p0);
    facets_raw_normals_.emplace_back(cross_product);

    //std::cout << "Normal facets\n" << facets_normals_.back();

    //glm::vec3 pstart = compute_facet_center(p0, p1, p2);
    //glm::vec3 pend = pstart + vnormal;
    //glm::vec4 pstart4{pstart[0], pstart[1], pstart[2], 1.0f};
    //glm::vec4 pend4{pend[0], pend[1], pend[2], 1.0f};

    //facets_normals_starts_.push_back(pstart4);
    //facets_normals_ends_.push_back(pend4);
  }
}
#endif

bool StlMesh::check() {
  bool status = false;

  int error_count = 0;
  int warn_count = 0;
  //
  // check index of facets is consistent with the list of vertices
  // and the list of textures
  size_t vmin_idx = std::numeric_limits<size_t>::max();
  size_t vmax_idx = std::numeric_limits<size_t>::min();

  // size_t tmin_idx = std::numeric_limits<size_t>::max();
  // size_t tmax_idx = std::numeric_limits<size_t>::min();

  for (auto &afacet : facets_) {
    auto [min_idx, max_idx] = afacet.vertex_min_max_idx();
    if (min_idx < vmin_idx) {
      vmin_idx = min_idx;
    }
    if (max_idx > vmax_idx) {
      vmax_idx = max_idx;
    }
  }

  if (vmin_idx != 0) {
    ++warn_count;
    std::cout << "Warning: vertex index up to " << vmin_idx << " are not used"
              << std::endl;
  }
  if (vmax_idx < (vertices_.size() - 1)) {
    ++warn_count;
    std::cout << "Warning: vertex index from " << vmax_idx << " are not used"
              << std::endl;
  }
  if (vmax_idx > (vertices_.size() - 1)) {
    ++error_count;
    std::cout << "Error: vertex index greater than " << vmax_idx
              << " are not defined" << std::endl;
  }

  if (warn_count > 0) {
    std::cout << "Found " << warn_count << " warnings" << std::endl;
  }

  if (error_count == 0) {
    status = true;
  }
  return status;
}

void StlMesh::info() {
  logger::info("Mesh '{0}' loaded in {1:d} ms with {2} vertices and {3} facets",
               get_name(), load_time(), Util::commify(vertices_.size()),
               Util::commify(facets_.size()));
}

void StlMesh::add_vertex(glm::vec4 const &v4d) { vertices_.push_back(v4d); }

void StlMesh::add_vertex(float x, float y, float z) {
  vertices_.emplace_back(x, y, z, 1.0f);
}

void StlMeshFactory::add_mesh(const std::string &name, StlMesh &mesh) {
  if (name2mesh_.count(name) > 0) {
    std::cout << "Warning adding a mesh with same name, deleting the old one"
              << name << std::endl;
    auto it = name2mesh_.find(name);
    // StlMesh* ptr = (*it).second;
    // delete ptr;
    name2mesh_.erase(it);
  }
  name2mesh_.insert_or_assign(name, std::move(mesh));
}

StlMesh &StlMeshFactory::get_mesh(const std::string &name) {
  if (name2mesh_.count(name) == 0) {
    std::cout << "Fatal error, get_mesh(), not mesh is not found for " << name
              << std::endl;
    exit(-1);
  }
  auto it = name2mesh_.find(name);
  StlMesh &mesh = (*it).second;
  return mesh;
}

//
// import STL file
// simple support
// 80 bytes header
// 4 bytes UINT32  number of triangles
//   --- group of 50 bytes: 12 + 12 + 12 + 12 + 2
//   float[3] normals
//   float[3] vertex0
//   float[3] vertex1
//   float[3] vertex2
//   uint16_t unused

StlMesh &StlMeshFactory::create_from_stlfile(const std::string &name,
                                             const std::string &file_name,
                                             const float scale_factor) {
  // std::cout << "[DEBUG] Parsing STL file " << name.c_str();

  auto time_start = std::chrono::high_resolution_clock::now();

  std::array<char, 80> rbuffer;

  std::filesystem::path p{file_name};

  if (not std::filesystem::is_regular_file(p)) {
    logger::error("The mesh file '{0}' can not be read", file_name);
    std::exit(1);
  }
  // auto parent_path = p.parent_path();

  StlMesh mesh{name};
  std::string mtllib{};
  std::string usemtl{};

  try {
    std::ifstream obj_strm(file_name, std::ios::in | std::ios::binary);
    obj_strm.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    // compute the size of the file
    obj_strm.seekg(0, std::ios_base::end);
    auto file_size = static_cast<size_t>(obj_strm.tellg());

    if (file_size < 84) {
      throw stlmesh::exception{"Error: STL file is incomplete"};
    }

    obj_strm.seekg(0, std::ios_base::beg);
    obj_strm.read(rbuffer.data(), rbuffer.size());

    std::string start_with{rbuffer[0], rbuffer[1], rbuffer[2], rbuffer[3],
                           rbuffer[4]};

    if (start_with == "solid") {
      throw stlmesh::exception{"Error: STL file is incomplete"};
    }

    // std::cout << "INFO STL file " << p.stem().string() << " is binary,
    // continue parsing" << std::endl; std::cout << "INFO STL file size " <<
    // file_size << std::endl;
    logger::info("Parsing binary STL file '.../{}' with {} bytes",
                 p.filename().string(), file_size);

    uint32_t number_of_triangles;
    obj_strm.read(reinterpret_cast<char *>(&number_of_triangles),
                  sizeof(number_of_triangles));

    // std::cout << "DEBUG number of triangles = " << number_of_triangles <<
    // std::endl;

    file_size -= 84;
    if ((50 * number_of_triangles) < file_size) {
      throw stlmesh::exception{"Error: file size is too large"};
    }
    if ((50 * number_of_triangles) > file_size) {
      throw stlmesh::exception{"Error: file size is big large"};
    }

    std::unordered_map<const ThreeFloats, size_t, HashV3D, KeyCompV3D> v3d2idx;

    // the number of unique vertices is ~ T/2
    // we can size the hash table to costly rehash

    v3d2idx.reserve(number_of_triangles / 2);

    size_t vidx = 0;
    // size_t cnt;
    size_t vidx0;
    size_t vidx1;
    size_t vidx2;
    float f0;
    float f1;
    float f2;

    for (size_t idx = 0; idx < number_of_triangles; ++idx) {
      // consume the normal
      obj_strm.read(rbuffer.data(), 12);

      // consume vertex 0
      obj_strm.read(reinterpret_cast<char *>(&f0), sizeof(f0));
      obj_strm.read(reinterpret_cast<char *>(&f1), sizeof(f1));
      obj_strm.read(reinterpret_cast<char *>(&f2), sizeof(f2));
      ThreeFloats v0{f0, f1, f2};

      auto it = v3d2idx.find(v0);
      if (it == v3d2idx.end()) {
        vidx0 = vidx++;
        v3d2idx.insert({v0, vidx0});
        mesh.add_vertex(f0 * scale_factor, f1 * scale_factor,
                        f2 * scale_factor);
      } else {
        vidx0 = it->second;
      }

      // consume vertex 1
      obj_strm.read(reinterpret_cast<char *>(&f0), sizeof(f0));
      obj_strm.read(reinterpret_cast<char *>(&f1), sizeof(f1));
      obj_strm.read(reinterpret_cast<char *>(&f2), sizeof(f2));
      ThreeFloats v1{f0, f1, f2};

      it = v3d2idx.find(v1);
      if (it == v3d2idx.end()) {
        vidx1 = vidx++;
        v3d2idx.insert({v1, vidx1});
        mesh.add_vertex(f0 * scale_factor, f1 * scale_factor,
                        f2 * scale_factor);
      } else {
        vidx1 = it->second;
      }

      // consume vertex 2
      obj_strm.read(reinterpret_cast<char *>(&f0), sizeof(f0));
      obj_strm.read(reinterpret_cast<char *>(&f1), sizeof(f1));
      obj_strm.read(reinterpret_cast<char *>(&f2), sizeof(f2));
      ThreeFloats v2{f0, f1, f2};

      it = v3d2idx.find(v2);
      if (it == v3d2idx.end()) {
        vidx2 = vidx++;
        v3d2idx.insert({v2, vidx2});
        mesh.add_vertex(f0 * scale_factor, f1 * scale_factor,
                        f2 * scale_factor);
      } else {
        vidx2 = it->second;
      }

      mesh.add_facet(vidx0, vidx1, vidx2);

      // consume the dummy two bytes
      obj_strm.read(rbuffer.data(), 2);
    }
  } catch (const stlmesh::exception &error) {
    std::cout << "ERROR: specific stl file error: " << error.what();
    exit(-1);
  } catch (const std::exception &error) {
    std::cout << "ERROR: generic stl file error: " << error.what();
    exit(-1);
  }

  /*
  pmesh->compute_facet_normal();
  pmesh->compute_vertex_normal();
  pmesh->set_default_image();

  if(mtllib.length() != 0) {
    std::filesystem::path mtllib_file_path = parent_path / mtllib;
    load_texture_file(pmesh, mtllib_file_path.string(), usemtl);
  }
  */

  // mesh.info();
  if (not mesh.check()) {
    std::cout << "Error: check mesh failure";
    exit(0);
  }
  mesh.compute_bbox();

  auto time_end = std::chrono::high_resolution_clock::now();
  auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      time_end - time_start);
  mesh.load_time(static_cast<uint64_t>(elapsed_time_ms.count()));

  add_mesh(name, mesh);
  return get_mesh(name);
}

void StlMesh::compute_bbox() {
  float xmin = std::numeric_limits<float>::max();
  float xmax = std::numeric_limits<float>::min();
  float ymin = std::numeric_limits<float>::max();
  float ymax = std::numeric_limits<float>::min();
  float zmin = std::numeric_limits<float>::max();
  float zmax = std::numeric_limits<float>::min();

  for (const auto &vertex : vertices_) {
    if (vertex[0] < xmin) {
      xmin = vertex[0];
    } else if (vertex[0] > xmax) {
      xmax = vertex[0];
    }

    if (vertex[1] < ymin) {
      ymin = vertex[1];
    } else if (vertex[1] > ymax) {
      ymax = vertex[1];
    }

    if (vertex[2] < zmin) {
      zmin = vertex[2];
    } else if (vertex[2] > zmax) {
      zmax = vertex[2];
    }
  }
  Bbox3d bbox{xmin, xmax, ymin, ymax, zmin, zmax};
  bbox_.swap(bbox);
}
