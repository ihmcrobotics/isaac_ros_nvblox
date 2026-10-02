// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 IHMC Robotics Lab
#pragma once
#include <memory>
#include <vector>
#include "nvblox_msgs/msg/mesh.hpp"
namespace nvblox {
// Kimera-PGMO compression -> mesh delta -> persistent CPU mesh.
// No pose optimization; accumulated archived geometry is not memory-bounded.
class RetainedMeshArchive {
 public:
  explicit RetainedMeshArchive(double resolution = 0.005);
  ~RetainedMeshArchive();
  void clear();
  void archive(const std::vector<nvblox_msgs::msg::Index3D>& indices);
  nvblox_msgs::msg::Mesh update(const nvblox_msgs::msg::Mesh& input,
      const std::vector<nvblox_msgs::msg::Index3D>& removed, uint64_t timestamp_ns,
      bool full_snapshot);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  double resolution_;
};
}  // namespace nvblox
