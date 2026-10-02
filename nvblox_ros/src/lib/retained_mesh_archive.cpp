// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 IHMC Robotics Lab
#include "nvblox_ros/retained_mesh_archive.hpp"
#include <kimera_pgmo/compression/delta_compression.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace nvblox {
namespace {
using Key = std::array<int, 3>;
using Block = nvblox_msgs::msg::MeshBlock;
Key key(const nvblox_msgs::msg::Index3D& i) {return {i.x,i.y,i.z};}
// PGMO consumes a triangle soup, not the indexed nvblox vertex array.
class Blocks : public kimera_pgmo::MeshInterface {
 public:
  std::map<Key, Block> data;
  kimera_pgmo::BlockIndices indices;
  mutable const Block* current = nullptr;
  const kimera_pgmo::BlockIndices& blockIndices() const override {return indices;}
  void markBlockActive(const kimera_pgmo::BlockIndex& i) const override {
    current = &data.at({i.x(),i.y(),i.z()});
  }
  size_t activeBlockSize() const override {return current->triangles.size();}
  pcl::PointXYZRGBA getActiveVertex(size_t i) const override {
    const auto v = current->triangles.at(i);
    const auto& p = current->vertices.at(v);
    pcl::PointXYZRGBA out; out.x=p.x; out.y=p.y; out.z=p.z;
    out.r=out.g=out.b=128; out.a=255;
    if (!current->colors.empty()) {
      const auto& c=current->colors.at(v);
      const auto byte=[](float x){return static_cast<uint8_t>(255*std::clamp(x,0.0f,1.0f));};
      out.r=byte(c.r);out.g=byte(c.g);out.b=byte(c.b);out.a=byte(c.a);
    }
    return out;
  }
  Ptr clone() const override {return std::make_shared<Blocks>(*this);}
};
void validate(const Block& b) {
  if (b.triangles.size()%3 || (!b.colors.empty() && b.colors.size()!=b.vertices.size()))
    throw std::invalid_argument("Invalid nvblox mesh dimensions");
  for (auto i:b.triangles) {
    if(i<0 || static_cast<size_t>(i)>=b.vertices.size()) throw std::invalid_argument("Invalid mesh index");
    const auto& p=b.vertices[i];
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) throw std::invalid_argument("Nonfinite mesh vertex");
    if(!b.colors.empty()) {const auto& c=b.colors[i];
      if(!std::isfinite(c.r)||!std::isfinite(c.g)||!std::isfinite(c.b)||!std::isfinite(c.a)) throw std::invalid_argument("Nonfinite mesh color");}
  }
}
}
struct RetainedMeshArchive::Impl {
  explicit Impl(double r): compressor(r) {}
  kimera_pgmo::DeltaCompression compressor;
  pcl::PointCloud<pcl::PointXYZRGBA> vertices;
  std::vector<kimera_pgmo::Timestamp> stamps;
  std::vector<pcl::Vertices> faces;
  std::set<Key> active_blocks, previous_active_regions;
  std::map<Key,std::vector<size_t>> archived_regions;
  size_t archived_faces=0;
  float block_size=0;
  std::string frame;
  Key region(size_t face) const {
    const auto& f=faces.at(face).vertices;
    std::array<double,3> center{};
    for(auto i:f) {const auto& p=vertices.at(i);center[0]+=p.x/3.;center[1]+=p.y/3.;center[2]+=p.z/3.;}
    Key k;
    for(size_t i=0;i<3;++i) {
      const auto v=std::floor(center[i]/block_size);
      if(v<std::numeric_limits<int>::min()||v>std::numeric_limits<int>::max()) throw std::overflow_error("Mesh region index overflow");
      k[i]=static_cast<int>(v);
    }
    return k;
  }
  Block block(const std::vector<size_t>& frozen,const std::vector<size_t>& active) const {
    Block out;std::map<uint32_t,int32_t> remap;
    const auto append=[&](const std::vector<size_t>& list) {
      for(auto fi:list) for(auto vi:faces.at(fi).vertices) {
        auto [it,inserted]=remap.emplace(vi,static_cast<int32_t>(out.vertices.size()));
        if(inserted) {
          const auto& p=vertices.at(vi);geometry_msgs::msg::Point32 v;v.x=p.x;v.y=p.y;v.z=p.z;
          std_msgs::msg::ColorRGBA c;c.r=p.r/255.f;c.g=p.g/255.f;c.b=p.b/255.f;c.a=p.a/255.f;
          out.vertices.push_back(v);out.colors.push_back(c);
        }
        out.triangles.push_back(it->second);
      }
    };append(frozen);append(active);return out;
  }
};
RetainedMeshArchive::RetainedMeshArchive(double r):resolution_(r) {
  if(!std::isfinite(r)||r<=0) throw std::invalid_argument("Compression resolution must be positive");
  clear();
}
RetainedMeshArchive::~RetainedMeshArchive()=default;
void RetainedMeshArchive::clear(){impl_=std::make_unique<Impl>(resolution_);}
void RetainedMeshArchive::archive(const std::vector<nvblox_msgs::msg::Index3D>& indices) {
  kimera_pgmo::BlockIndices outgoing;
  for(const auto& i:indices) if(impl_->active_blocks.erase(key(i))) outgoing.emplace_back(i.x,i.y,i.z);
  impl_->compressor.clearArchivedBlocks(outgoing);
}
nvblox_msgs::msg::Mesh RetainedMeshArchive::update(const nvblox_msgs::msg::Mesh& input,
    const std::vector<nvblox_msgs::msg::Index3D>& removed,uint64_t stamp,bool full) {
  if(input.blocks.size()!=input.block_indices.size() || !std::isfinite(input.block_size_m) || input.block_size_m<=0)
    throw std::invalid_argument("Invalid mesh message");
  for(const auto& b:input.blocks) validate(b);
  if(input.clear) {clear();full=true;}
  auto& s=*impl_;
  if(s.block_size!=0 && (s.block_size!=input.block_size_m || s.frame!=input.header.frame_id))
    throw std::invalid_argument("Mesh frame or block size changed without reset");
  s.block_size=input.block_size_m;s.frame=input.header.frame_id;
  Blocks blocks;
  // GPU eviction after explicit archival must not delete frozen mesh. Other
  // deletions replace active observations with an empty block in the compressor.
  for(const auto& i:removed) if(s.active_blocks.erase(key(i))) blocks.data[key(i)]=Block{};
  for(size_t j=0;j<input.blocks.size();++j) {
    const auto k=key(input.block_indices[j]);blocks.data[k]=input.blocks[j];s.active_blocks.insert(k);
  }
  for(const auto& [k,b]:blocks.data) blocks.indices.emplace_back(k[0],k[1],k[2]);
  const auto delta=s.compressor.update(blocks,stamp);
  delta->updateMesh(s.vertices,s.stamps,s.faces);
  std::set<Key> dirty=s.previous_active_regions;
  const auto archived=delta->getTotalArchivedFaces();
  for(size_t f=s.archived_faces;f<archived;++f) {auto k=s.region(f);s.archived_regions[k].push_back(f);dirty.insert(k);}
  s.archived_faces=archived;
  std::map<Key,std::vector<size_t>> active;
  s.previous_active_regions.clear();
  for(size_t f=archived;f<s.faces.size();++f) {auto k=s.region(f);active[k].push_back(f);dirty.insert(k);s.previous_active_regions.insert(k);}
  if(full) for(const auto& [k,v]:s.archived_regions) dirty.insert(k);
  nvblox_msgs::msg::Mesh out;out.header=input.header;out.block_size_m=input.block_size_m;out.clear=full;
  const std::vector<size_t> empty;
  for(const auto& k:dirty) {
    auto a=s.archived_regions.find(k);auto b=active.find(k);
    auto mesh=s.block(a==s.archived_regions.end()?empty:a->second,b==active.end()?empty:b->second);
    nvblox_msgs::msg::Index3D i;i.x=k[0];i.y=k[1];i.z=k[2];out.block_indices.push_back(i);out.blocks.push_back(std::move(mesh));
  }
  return out;
}
}  // namespace nvblox
