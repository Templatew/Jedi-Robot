#include "facet.h"

Facet::Facet(size_t v0, size_t v1, size_t v2) : vertices_{v0, v1, v2} {}

Facet::Facet(size_t v0, size_t v1, size_t v2, size_t t0, size_t t1, size_t t2)
    : vertices_{v0, v1, v2}, uv_vertices_{t0, t1, t2} {}
