#include "nvblox_ros/retained_mesh_archive.hpp"
#include <cassert>
#include <vector>
struct Point { float x=0,y=0,z=0; };
struct Color {float r=0,g=0,b=0,a=0;};
struct Block {std::vector<Point> vertices,normals;std::vector<Color> colors;std::vector<int> triangles;};
Block triangle(float x) {Block b;b.vertices={{x,0,0},{x+1,0,0},{x,1,0}};b.triangles={0,1,2};return b;}
int main(){
 nvblox::RetainedMeshArchive<Block> archive;
 const std::array<int,3> k{-1,0,2};
 archive.update(k,triangle(0));archive.freeze(k);archive.removeActive(k);
 assert(archive.combined(k).triangles.size()==3);
 archive.update(k,{});assert(archive.combined(k).triangles.size()==3);
 archive.update(k,triangle(2));assert(archive.combined(k).triangles.size()==6);
 archive.update(k,triangle(3));assert(archive.combined(k).triangles.size()==6);
 archive.update(k,triangle(0));assert(archive.combined(k).triangles.size()==3);
 archive.update(k,triangle(2));archive.freeze(k);archive.removeActive(k);
 assert(archive.combined(k).triangles.size()==6);
 archive.update({2,0,0},triangle(8));archive.removeActive({2,0,0});
 assert(archive.combined({2,0,0}).triangles.empty());
 archive.clear();assert(archive.combined(k).triangles.empty());
}
