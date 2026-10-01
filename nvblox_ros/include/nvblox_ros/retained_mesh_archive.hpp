// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 IHMC Robotics Lab
#pragma once

#include <algorithm>
#include <array>
#include <map>
#include <set>

namespace nvblox {
// Experimental active/frozen lifecycle. Archival freezes geometry before voxel
// clearing; a new observation at the same spatial key starts a new active mesh.
// This is not a TSDF archive and does not remove stale frozen surfaces.
template<class Block>
class RetainedMeshArchive {
 public:
  using Key = std::array<int, 3>;
  void clear() { active_.clear(); frozen_.clear(); }
  void update(const Key& key, const Block& block) { active_[key] = block; }
  void freeze(const Key& key) {
    const auto it = active_.find(key);
    if (it == active_.end()) return;
    frozen_[key] = combined(key);
    active_.erase(it);
  }
  void removeActive(const Key& key) { active_.erase(key); }
  Block combined(const Key& key) const {
    const auto a = active_.find(key);
    const auto f = frozen_.find(key);
    if (f == frozen_.end()) return a == active_.end() ? Block{} : a->second;
    if (a == active_.end() || a->second.triangles.empty()) return f->second;
    Block output;
    std::set<std::array<float, 9>> seen;
    // New colors win for exactly coincident triangles. No spatial welding:
    // different surfaces must not be joined across block boundaries.
    const auto append = [&](const Block& source) {
      for (size_t t = 0; t + 2 < source.triangles.size(); t += 3) {
        std::array<std::array<float, 3>, 3> points;
        for (size_t j = 0; j < 3; ++j) {
          const auto& p = source.vertices.at(source.triangles.at(t + j));
          points[j] = {p.x, p.y, p.z};
        }
        std::sort(points.begin(), points.end());
        std::array<float, 9> signature;
        for (size_t j = 0; j < 3; ++j)
          std::copy(points[j].begin(), points[j].end(), signature.begin() + 3*j);
        if (!seen.insert(signature).second) continue;
        for (size_t j = 0; j < 3; ++j) {
          const auto index = source.triangles.at(t + j);
          output.triangles.push_back(output.vertices.size());
          output.vertices.push_back(source.vertices.at(index));
          typename decltype(output.colors)::value_type color;
          color.r = color.g = color.b = 0.5f; color.a = 1.0f;
          output.colors.push_back(source.colors.empty() ? color : source.colors.at(index));
          typename decltype(output.normals)::value_type normal;
          output.normals.push_back(source.normals.empty() ? normal : source.normals.at(index));
        }
      }
    };
    append(a->second);
    append(f->second);
    return output;
  }
 private:
  std::map<Key, Block> active_, frozen_;
};
}  // namespace nvblox
