# racecar_description (copied assets)

These files are copies of the `racecar_description` package of the public repository
[SherbyRobotics/racecar](https://github.com/SherbyRobotics/racecar), at commit
`0878b75db5786868ae8f950dceb109506eaf17e7` (2026-02-25, branch `ros2`), which is the last
commit that touches the package. The repository is under the MIT License, Copyright (c)
2019 Alexandre Girard (Université de Sherbrooke); the copy of that licence is in
`LICENSE` and it covers these files. The URDF structure and the meshes (chassis plate,
wheels, 1 cm hinge spheres, `hokuyo.dae`) appear to derive from the MIT RACECAR
`racecar_description` package, also MIT-licensed.

They are shipped inside the Python package (`minilink/graphical/assets/`) so the skin works from a
plain install, with no ROS workspace and no environment variable to set: 716 KB in total,
12 files.

| file | size | sha256 (first 12) |
|---|---|---|
| `urdf/racecar.xacro` | 8.3 kB | `2e7edfaa24a8` |
| `urdf/macros.xacro` | 4.5 kB | `a18e7e1be985` |
| `urdf/materials.xacro` | 0.9 kB | `881c5e0c8ce3` |
| `urdf/racecar.gazebo` | 5.8 kB | `34d9d27e049c` |
| `meshes/chassis.STL` | 16 kB | `0fb5181efb2c` |
| `meshes/left_front_wheel.STL` | 82 kB | `9c5001eb5bf5` |
| `meshes/left_rear_wheel.STL` | 82 kB | `fb2bf2ff507f` |
| `meshes/right_front_wheel.STL` | 82 kB | `e0f01172160f` |
| `meshes/right_rear_wheel.STL` | 82 kB | `1a03985b91e0` |
| `meshes/left_steering_hinge.STL` | 118 kB | `971ee2e0d326` |
| `meshes/right_steering_hinge.STL` | 118 kB | `784a543eef92` |
| `meshes/hokuyo.dae` | 88 kB | `a42fae2752bf` |

`urdf/racecar.gazebo` is kept although nothing here simulates Gazebo: `racecar.xacro`
includes it, so dropping it would change what the expansion produces. The checksums are
checked by `tests/test_racecar_skin.py`, which fails if an asset is edited in place.

`urdf/racecar.urdf` is generated, not copied, and carries no checksum for that reason:

```python
write_urdf(racecar_urdf(), path, mesh_prefix="package://racecar_description/meshes/")
```

reproduces it byte for byte (a test checks that). The prefix matters: expansion resolves
`$(find racecar_description)` against the filesystem, so without it the written file would
carry the absolute paths of whoever regenerated it. On links, joints, visuals and
materials, the expansion matches what ROS `xacro` produces from the same files.

The STL meshes are binary and in metres. The chassis spans x from -0.04 to 0.365, y ±0.10
and z ±0.03. A wheel has radius 0.05 and width 0.045, extruded outward from its joint
plane. The COLLADA meshes of the package (`*.dae` other than `hokuyo.dae`) were not copied
because the URDF uses only the STL files.
