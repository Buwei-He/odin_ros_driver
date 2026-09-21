# This fork vs. upstream — read this before touching anything here

`origin` = `Buwei-He/odin_ros_driver` (this fork). `upstream` = `manifoldsdk/odin_ros_driver`
(Manifold's own repo, closed-source SDK underneath it). We're based on upstream's
`v0.14.2` release (`34d36d6`), squashed on top as one commit:
`6ca310a "ROS1 port + RGB-D virtual camera + real-hardware fixes"`.

**Purpose of this file**: when Manifold ships a new upstream version, this is the map
of everything we changed and why, so a human or a coding agent can tell in one read
which of our changes need to be re-checked or re-applied, without re-deriving the
diff from scratch. It's a map, not a copy of the diff — the reasoning for each fix
already lives as a comment at the point of the change (dated, root-caused); this file
just tells you where to look.

## What we added (net-new capability)

**ROS1 port.** Upstream is ROS2-only (`ament_cmake`, colcon). We build it under ROS1
Noetic (`catkin`) because that's what runs on the robot (ARI's own stack is ROS1).
`CMakeLists.txt` and `package.xml` carry both build paths now, gated on which ROS
distro is sourced — check `CMakeLists.txt`'s top-level `if` before assuming either
path is dead code.

**RGB-D virtual camera** (`sensor2rgbd.hpp/.cpp`, new files; `pointcloud_depth_converter.hpp/.cpp`,
substantially rewritten). Upstream's depth pipeline produces a lidar-frame point
cloud. We added a second output: the depth+color pair reprojected into a virtual
pinhole camera's image plane (`camera_0`), aligned and undistorted, at a fixed
output resolution — the format `percorso_perception_ros`'s DAAAM consumer expects.
This is the biggest single piece of new code (`pointcloud_depth_converter` diff is
~110 lines changed, `sensor2rgbd` is ~120 lines net new). `output_width`/
`output_height` in `PointCloudToDepthConverter`'s config **must** match what's
downstream, or the RGB and depth streams stop being pixel-aligned — see the comment
right above those two fields in `pointcloud_depth_converter.hpp`.

Published on `/odin1/rgbd/color`, `/odin1/rgbd/camera_info`,
`/odin1/depth_img_competetion` — gated behind `publish_rgbd:=true` +
`register_keys/senddepth:=1` (both required; `config/control_command.yaml` ships
`senddepth` off, so it must be overridden by whatever launches this — see
`launch_ROS1/odin1_ros1_rgbd.launch`). New nodes/executables:
`pcd2depth_node`, `cloud_reprojection_node`, `image_overlay_node` — upstream built
these but never `install()`-ed them (`CMakeLists.txt` fix), so a prior deploy of
upstream's own build would silently have shipped `host_sdk_sample` only.

## Real-hardware fixes (bugs upstream doesn't have fixed yet, as of v0.14.2)

- **RGB decode SIGSEGV** (`host_sdk_sample.h`/`.cpp`, `config/control_command.yaml`).
  `image.format` from the device is unreliable — observed reporting NV12 while
  actually streaming MJPEG, so the NV12 decode path read past the end of a much
  smaller JPEG buffer. Fixed by sniffing the real format from the data's own magic
  bytes (JPEG starts `FF D8`) instead of trusting the field. Full root-cause comment
  is inline right above the fix in `host_sdk_sample.h`.
- **`set_rgb_parameter` gate** (`host_sdk_sample.cpp`, `control_command.yaml`,
  `yaml_parser.h`). `lidar_set_rgb_parameter()` errors out against our
  device/firmware (v0.14.1) and leaves the device streaming frames that crash the
  decode path. New opt-in config key, default off — preserves pre-v0.14.2 behavior
  of never calling it. **Check this first if upstream changes RGB parameter
  handling**: if a newer firmware/SDK fixes the underlying call, this whole gate
  might become unnecessary.
- **USB/SDK fixes** already present in upstream's own history before our fork point
  (`6f993cc`, `d2bf605`) — not ours, just noting they predate the squash so `git log`
  doesn't look like it's missing them.

## Upgrade workflow, next time Manifold releases a new version

```bash
git fetch upstream
git log upstream/main --oneline          # what's new since our v0.14.2 base
git diff upstream/main --stat             # should mostly match the file list above —
                                           # any NEW file in this list vs. what's here
                                           # is something upstream touched that we did too
```

For each file both sides touched: `git diff 34d36d6 upstream/main -- <file>` shows
what upstream changed since our base, `git diff upstream/main -- <file>` (current)
shows what we still differ by. Decide per file: upstream's change already covers
what we needed (drop our patch), or it doesn't (keep ours, re-check it still applies
cleanly on top).

The one deliberately load-bearing check: **does upstream's new version still leave
`image.format` unreliable?** If Manifold fixes the SDK-side bug, the magic-byte
sniff in `host_sdk_sample.h` becomes redundant (harmless to keep, but worth noting
if it stops matching reality) and `set_rgb_parameter` may be safe to flip on.

## Uncommitted, worth knowing about

`launch_ROS1/odin1_ros1_rgbd.launch` had a punctuation-only comment fix pending as
of 2026-09-21 (`--` → `:`), harmless, not yet committed.
