# Local-map archival experiment

This branch separates active block geometry from frozen geometry, following the
active/archive lifecycle used by Hydra. It does not import Hydra or Kimera-PGMO
and does not reproduce their mesh-index compression or deformation handling.

- Mesh outgoing blocks before clearing their GPU voxels (existing behavior).
- Freeze their CPU triangles before releasing the voxel evidence.
- Replace active geometry on each update; do not replace frozen geometry merely
  because the same block coordinate is observed again.
- Publish the union of frozen and current triangles under the original block key.
- Suppress exactly coincident triangles, preferring current colors. No approximate
  vertex welding or cross-block merging is performed.
- Explicit active-block removal retains frozen geometry; reset clears both.
- Late subscribers receive the same composed geometry through full archive replay.

This deliberately favors geometry retention. Changed scenes can leave stale or
nearly coincident surfaces. Repeated revisits can increase CPU RAM, message size
and render cost. It does not reload TSDF evidence or prove that all holes originate
in the archive. Compare the same recording against the previous branch commit.
No voxel resolution, fusion parameters, clearing radius or RDX ABI were changed.

Standalone contract check (no ROS/GPU required):

```bash
c++ -std=c++17 -Wall -Wextra -Werror -I nvblox_ros/include \
  nvblox_ros/test/retained_mesh_archive_test.cpp -o /tmp/archive-test
/tmp/archive-test
```

Checks cover outgoing clearing, empty/partial revisits, active replacement,
coincident triangles, repeat archival, negative coordinates and reset.
