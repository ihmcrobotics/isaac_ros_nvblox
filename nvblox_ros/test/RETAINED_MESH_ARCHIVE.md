# Standalone compressed mesh archive

The local-map publisher uses the installed Kimera-PGMO `DeltaCompression` library.
No Hydra scene graph, GVD extraction or pose-optimization processing is started.
The existing Kimera-PGMO library links GTSAM/RPGO as build/runtime dependencies.
No third-party source is copied.

Pipeline:

1. Adapt changed indexed nvblox mesh blocks into PGMO's triangle-soup interface.
2. Compress at 0.005 m (Hydra's configured mesh-compression resolution, distinct
   from the 0.1 m TSDF voxel size).
3. Apply MeshDelta vertex/face updates to a persistent indexed CPU mesh.
4. Before GPU clearing, finalize outgoing mesh, archive its compressor blocks,
   and apply the pending archival delta.
5. Group output faces by spatial centroid into nvblox mesh-message blocks.
   Process newly archived faces once; regenerate active regions and regions that
   became empty. Previously frozen regions are not republished unless affected
   or a subscriber needs a full snapshot.
6. Publish through the existing RViz/RDX interfaces. Output block membership can
   differ from source TSDF block membership because compression shares vertices.

An empty active block or deletion updates the compressor; a post-archival GPU
removal does not erase frozen faces. Reset clears both the compressor and CPU
mesh. Repeated observations of active geometry replace it instead of accumulating
independent triangle copies. The compressor's own archival/revisit behavior is
used without adding the prior exact-triangle union policy.

Limitations:

- Archived geometry still accumulates in RAM; no disk eviction or hard RAM budget.
- Compression revisits all active geometry and adds CPU cost; no speedup claimed.
- Archived geometry is not re-fused TSDF evidence. Changed scenes may leave stale
  geometry. This does not guarantee removal of holes or duplicate revisit surfaces.
- No deformation or loop-closure optimization. Output normals are omitted, as
  supported by the existing unlit RGB RViz and RDX viewers.
- Input fusion settings, 4 m integration distance, 8 m clearing radius and native
  RDX snapshot ABI are unchanged.

Build the image normally with `./build_image.sh`, then restart `./run_nvblox.sh`.
The nvblox Docker stage sources the scene-graph underlay to find Kimera-PGMO.

With BUILD_TESTING enabled, CTest target `retained_mesh_archive_test` checks actual
PGMO compression, shared vertices, partial archival, empty/partial revisits,
active replacement, incremental viewer reconstruction, late-join replay, invalid
input, repeated archival, negative coordinates and reset. It requires the installed
ROS message and PGMO libraries but no GPU execution. Live recording validation and
CPU/RAM profiling are still required.
