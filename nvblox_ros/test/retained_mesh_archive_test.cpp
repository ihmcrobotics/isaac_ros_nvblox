// SPDX-License-Identifier: Apache-2.0
#include "nvblox_ros/retained_mesh_archive.hpp"
#include <cassert>
#include <map>
#include <array>
#include <iostream>
using Mesh=nvblox_msgs::msg::Mesh;
using Block=nvblox_msgs::msg::MeshBlock;
using Key=std::array<int,3>;
nvblox_msgs::msg::Index3D index(int x){nvblox_msgs::msg::Index3D i;i.x=x;return i;}
Block triangle(float x) {
 Block b;for(auto xy:std::array<std::array<float,2>,3>{{{x,0},{x+.2f,0},{x,.2f}}}) {
 geometry_msgs::msg::Point32 p;p.x=xy[0];p.y=xy[1];b.vertices.push_back(p);
 std_msgs::msg::ColorRGBA c;c.r=1;c.a=1;b.colors.push_back(c);
 } b.triangles={0,1,2};return b;
}
Mesh input(){Mesh m;m.block_size_m=.8f;m.header.frame_id="world";return m;}
struct Viewer {
 std::map<Key,Block> blocks;
 void apply(const Mesh& m){if(m.clear)blocks.clear();assert(m.blocks.size()==m.block_indices.size());for(size_t j=0;j<m.blocks.size();++j){auto i=m.block_indices[j];Key k{i.x,i.y,i.z};if(m.blocks[j].triangles.empty())blocks.erase(k);else blocks[k]=m.blocks[j];}}
 size_t faces()const{size_t n=0;for(auto& [k,b]:blocks){assert(b.colors.size()==b.vertices.size());for(auto i:b.triangles)assert(i>=0&&size_t(i)<b.vertices.size());n+=b.triangles.size()/3;}return n;}
};
int main(){nvblox::RetainedMeshArchive a;Viewer v;uint64_t t=0;
 auto send=[&](Mesh m,std::vector<nvblox_msgs::msg::Index3D> removed={},bool full=false){auto out=a.update(m,removed,++t,full);v.apply(out);return out;};
 auto m=input();m.block_indices={index(-1)};m.blocks={triangle(-.5f)};send(m);assert(v.faces()==1);
 // Duplicate triangles and nearby vertices compress within the active map.
 auto duplicate=triangle(-.5f);duplicate.triangles={0,1,2,0,1,2};m.blocks={duplicate};send(m);assert(v.faces()==1);
 a.archive({index(-1)});send(input());send(input(),{index(-1)});assert(v.faces()==1);
 m.blocks={Block{}};send(m);assert(v.faces()==1);
 m.blocks={triangle(-.2f)};send(m);assert(v.faces()==2);
 // Active replacement does not accumulate the preceding active observation.
 m.blocks={triangle(-.1f)};send(m);assert(v.faces()==2);
 a.archive({index(-1)});send(input());send(input(),{index(-1)});assert(v.faces()==2);
 Viewer late;late.apply(send(input(),{},true));assert(late.faces()==v.faces());
 m.block_indices={index(4)};m.blocks={triangle(3.3f)};send(m);assert(v.faces()==3);
 send(input(),{index(4)});assert(v.faces()==2);
 // A malformed update must not mutate the existing mesh.
 m.blocks[0].triangles={99,1,2};bool failed=false;try{send(m);}catch(const std::invalid_argument&){failed=true;}assert(failed);
 late.apply(send(input(),{},true));assert(late.faces()==2);
 auto reset=input();reset.clear=true;send(reset);assert(v.faces()==0);
 a.clear();send(input(),{},true);assert(v.faces()==0);
 // Shared vertices across neighboring input blocks survive partial archival.
 m=input();m.block_indices={index(0),index(1)};
 auto neighbor=triangle(0);neighbor.vertices[2].y=-.2f;
 m.blocks={triangle(0),neighbor};send(m);assert(v.faces()==2);
 a.archive({index(0)});send(input());send(input(),{index(0)});assert(v.faces()==2);
 m.block_indices={index(1)};m.blocks={triangle(.3f)};send(m);assert(v.faces()==2);
 a.archive({index(1)});send(input());assert(v.faces()==2);
 late.apply(send(input(),{},true));assert(late.faces()==2);
 // Many update/archive cycles preserve indices and replay equivalence.
 for(int i=2;i<32;++i) {
   m=input();m.block_indices={index(i)};m.blocks={triangle(float(i))};send(m);
   a.archive({index(i)});send(input());send(input(),{index(i)});
   assert(v.faces()==size_t(i+1));
 }
 late.apply(send(input(),{},true));assert(late.faces()==v.faces());
 std::cout<<"Compression, archival, deltas, replay, deletion and reset passed\n";
}
