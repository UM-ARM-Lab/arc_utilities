#pragma once

#include <cstdint>
#include <shape_msgs/MeshTriangle.h>

namespace arc_utilities {
namespace rmb {
shape_msgs::MeshTriangle MakeMeshTriangle(const uint32_t v1, const uint32_t v2, const uint32_t v3) {
  shape_msgs::MeshTriangle t;
  t.vertex_indices = {v1, v2, v3};
  return t;
}

}  // namespace rmb
}  // namespace arc_utilities

