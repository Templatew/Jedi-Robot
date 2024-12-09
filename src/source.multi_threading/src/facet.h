#pragma once

#include <vector>
#include <cstdint>
#include <tuple>
#include <algorithm>

/*
** A Facet is a triangle
** with lines
**      v0
**     /   \
**    /     \
**   v1 ---- v2
**
*/

class Facet {
 private:
  std::vector<size_t> vertices_;
  std::vector<size_t> uv_vertices_;

 public:
  Facet(size_t v0, size_t v1, size_t v2);
  Facet(size_t v0, size_t v1, size_t v2, size_t t0, size_t t1, size_t t2 );

  bool has_texture() const {
    return not uv_vertices_.empty();
  }

  template<size_t idx>
  size_t get() const {
    return vertices_[idx];
  }

  template<size_t idx>
  size_t get_textures() const {
    return uv_vertices_[idx];
  }

  std::pair<size_t, size_t>  vertex_min_max_idx() {
    auto p = std::ranges::minmax(vertices_);
    return {p.min, p.max};
  }

  std::pair<size_t, size_t>  texture_min_max_idx() {
    auto p = std::ranges::minmax(uv_vertices_);
    return {p.min, p.max};
  }
};
