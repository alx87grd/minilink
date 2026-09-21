"""Contract of the URDF assets, the placement, the looks, and the plant that drives them."""

import hashlib

import numpy as np
import pytest

from minilink.catalog import UdeSRacecar, UdeSRacecarDyn, UdeSRacecarDyn3D
from minilink.graphical.catalog.racecar_skin import (
    ASSETS,
    GEOMETRY,
    MESH_DIR,
    ackermann_angles,
    racecar_frames,
    racecar_skin_2d,
    racecar_skin_3d,
    racecar_urdf,
    track_lines,
    track_scene,
    urdf_skin,
)
from minilink.graphical.urdf import write_urdf
from minilink.planning.spatial.paths import circuit_waypoints

URDF_LINKS = (
    "base_footprint",
    "base_link",
    "chassis",
    "left_rear_wheel",
    "right_rear_wheel",
    "left_steering_hinge",
    "right_steering_hinge",
    "left_front_wheel",
    "right_front_wheel",
    "base_laser",
    "camera_link",
)

# sha256 prefixes of the copied assets, as listed in their README
CHECKSUMS = {
    "urdf/racecar.xacro": "2e7edfaa24a8",
    "urdf/macros.xacro": "a18e7e1be985",
    "urdf/materials.xacro": "881c5e0c8ce3",
    "urdf/racecar.gazebo": "34d9d27e049c",
    "meshes/chassis.STL": "0fb5181efb2c",
    "meshes/left_front_wheel.STL": "9c5001eb5bf5",
    "meshes/left_rear_wheel.STL": "fb2bf2ff507f",
    "meshes/right_front_wheel.STL": "e0f01172160f",
    "meshes/right_rear_wheel.STL": "1a03985b91e0",
    "meshes/left_steering_hinge.STL": "971ee2e0d326",
    "meshes/right_steering_hinge.STL": "784a543eef92",
    "meshes/hokuyo.dae": "a42fae2752bf",
}


# The assets


@pytest.mark.parametrize("name", sorted(CHECKSUMS))
def test_the_copied_asset_is_the_one_that_was_published(name):
    """Every URDF and mesh file still hashes to what its README says it does."""
    digest = hashlib.sha256((ASSETS / name).read_bytes()).hexdigest()

    assert digest[:12] == CHECKSUMS[name]


def test_the_assets_carry_their_licence_and_their_provenance():
    """A published copy travels with its MIT licence and the commit it came from."""
    licence = (ASSETS / "LICENSE").read_text()
    readme = (ASSETS / "README.md").read_text()

    assert "MIT License" in licence
    assert "Copyright (c) 2019 Alexandre Girard" in licence
    assert "SherbyRobotics/racecar" in readme
    assert "0878b75db5786868ae8f950dceb109506eaf17e7" in readme
    for name, digest in CHECKSUMS.items():
        assert digest in readme, name


def test_the_flat_urdf_is_reproducible_and_carries_no_local_path(tmp_path):
    """The generated ``racecar.urdf`` comes back byte for byte, with ``package://`` meshes."""
    shipped = (ASSETS / "urdf" / "racecar.urdf").read_text()
    written = tmp_path / "racecar.urdf"

    write_urdf(
        racecar_urdf(), written, mesh_prefix="package://racecar_description/meshes/"
    )

    assert written.read_text() == shipped
    assert 'mesh filename="package://' in shipped
    assert 'mesh filename="file://' not in shipped  # a path of whoever regenerated it
    assert str(ASSETS) not in shipped


def test_the_meshes_live_inside_the_package():
    """The skin needs no environment variable and no ROS workspace to find them."""
    assert ASSETS.parents[1].name == "graphical"
    assert ASSETS.parents[2].name == "minilink"
    assert MESH_DIR.is_dir()
    assert sorted(p.name for p in MESH_DIR.iterdir()) == sorted(
        name.split("/")[1] for name in CHECKSUMS if name.startswith("meshes/")
    )


# The geometry


def test_urdf_geometry_is_the_published_one():
    """Wheelbase, wheel radius and track come from the public URDF."""
    assert GEOMETRY["wheelbase"] == pytest.approx(0.34)
    assert GEOMETRY["wheel_radius"] == pytest.approx(0.05, abs=1e-6)
    assert GEOMETRY["kingpin_track"] == pytest.approx(0.20)


def test_the_drawn_car_has_the_dimensions_of_the_simulated_one():
    """The plant rolls on exactly the geometry the skin draws, so nothing is rescaled."""
    params = UdeSRacecarDyn().params

    assert params["a"] + params["b"] == pytest.approx(GEOMETRY["wheelbase"])
    assert params["r_f"] == pytest.approx(GEOMETRY["wheel_radius"], abs=1e-6)
    assert params["r_r"] == pytest.approx(GEOMETRY["wheel_radius"], abs=1e-6)


def test_the_lidar_sits_at_its_urdf_height():
    """``base_laser`` rides at the scan height above the ground, wheels included."""
    frames = racecar_frames(0.0, 0.0, 0.0, 0.0)

    height = np.asarray(frames["base_laser"])[2, 3]
    assert height == pytest.approx(GEOMETRY["lidar_height"])
    assert height == pytest.approx(0.216)
    assert height > GEOMETRY["wheel_radius"]


# Placement


def test_frames_carry_the_whole_robot():
    """Placement returns the bicycle frames and every URDF link, as valid poses."""
    frames = racecar_frames(1.0, 2.0, 0.3, 0.2)

    assert {"body", "axle_front"}.issubset(frames)
    assert set(URDF_LINKS).issubset(frames)
    for name, T in frames.items():
        R = np.asarray(T)[:3, :3]
        assert np.asarray(T).shape == (4, 4), name
        assert R @ R.T == pytest.approx(np.eye(3), abs=1e-9), name


def test_the_car_is_placed_where_it_is_asked_to_be():
    """``body`` sits at the pose given, and the axles ride at the wheel radius."""
    X, Y, psi, radius = 1.0, -2.0, 0.4, 0.05

    frames = racecar_frames(X, Y, psi, 0.0, wheel_radius=radius)

    assert np.asarray(frames["body"])[:2, 3] == pytest.approx([X, Y])
    assert np.asarray(frames["base_link"])[2, 3] == pytest.approx(radius)
    assert np.arctan2(*np.asarray(frames["body"])[1::-1, 0]) == pytest.approx(psi)


def test_reference_point_can_sit_at_the_centre_of_gravity():
    """``ref_to_rear_axle`` moves the robot back so the pose means the CG."""
    b = 0.17

    at_axle = racecar_frames(0.0, 0.0, 0.0, 0.0)
    at_cg = racecar_frames(0.0, 0.0, 0.0, 0.0, ref_to_rear_axle=b)

    shift = np.asarray(at_cg["base_footprint"])[0, 3]
    assert shift == pytest.approx(np.asarray(at_axle["base_footprint"])[0, 3] - b)


def test_placement_is_a_pure_function():
    """Two calls give the same poses, and neither disturbs the cached URDF tables."""
    first = racecar_frames(1.0, 2.0, 0.3, 0.25, phi_rear=1.0)
    other = racecar_frames(-4.0, 0.5, -1.2, -0.4, phi_rear=7.0, wheel_radius=0.06)
    again = racecar_frames(1.0, 2.0, 0.3, 0.25, phi_rear=1.0)

    assert other
    for name, T in first.items():
        assert np.asarray(again[name]) == pytest.approx(np.asarray(T)), name


def test_placement_runs_under_jax():
    """``tf`` is an equation path: it has to jit and differentiate like the model."""
    jax = pytest.importorskip("jax")

    def hinge_x(delta):
        return racecar_frames(0.0, 0.0, 0.0, delta)["left_front_wheel"][0, 3]

    jitted = jax.jit(hinge_x)(0.3)
    slope = jax.jacfwd(hinge_x)(0.3)

    assert float(jitted) == pytest.approx(hinge_x(0.3))
    assert np.isfinite(float(slope))


# Ackermann steering


def test_ackermann_turns_the_inner_wheel_more():
    """The inner wheel follows the tighter circle, and zero steer is symmetric."""
    left, right = ackermann_angles(
        0.35, GEOMETRY["wheelbase"], GEOMETRY["kingpin_track"]
    )
    straight = ackermann_angles(0.0, GEOMETRY["wheelbase"], GEOMETRY["kingpin_track"])

    assert left > 0.35 > right > 0.0
    assert straight[0] == pytest.approx(0.0)
    assert straight[1] == pytest.approx(0.0)


def test_both_front_wheels_turn_about_one_centre():
    """The two steering axes and the rear axle meet at a single point: no scrub."""
    delta, L, w = 0.35, GEOMETRY["wheelbase"], GEOMETRY["kingpin_track"]
    left, right = ackermann_angles(delta, L, w)

    # turn centre of the bicycle model: on the rear axle line, at y = L / tan(delta)
    centre = np.array([0.0, L / np.tan(delta)])
    for hub, angle in (([L, 0.5 * w], left), ([L, -0.5 * w], right)):
        arm = centre - np.asarray(hub)  # hub to centre is that wheel's own axle
        rolling = np.array([np.cos(angle), np.sin(angle)])  # where the wheel points
        assert abs(np.dot(arm / np.linalg.norm(arm), rolling)) < 1e-12


def test_the_hinges_are_turned_by_the_ackermann_angles():
    """The placement puts each wheel at its own angle, not at the bicycle angle."""
    delta = 0.4
    left, right = ackermann_angles(
        delta, GEOMETRY["wheelbase"], GEOMETRY["kingpin_track"]
    )

    frames = racecar_frames(0.0, 0.0, 0.0, delta)

    # A wheel points across its own axle, which is the local z of its URDF link. The
    # tolerance is the URDF's own: its joints are written with rpy 1.5708, 4e-6 off.
    for link, angle in (
        ("left_front_wheel", left),
        ("right_front_wheel", right),
        ("left_rear_wheel", 0.0),
        ("right_rear_wheel", 0.0),
    ):
        axle = np.asarray(frames[link])[:3, 2]
        assert np.arctan2(axle[0], -axle[1]) == pytest.approx(angle, abs=1e-5)
        assert axle[2] == pytest.approx(0.0, abs=1e-5)


# The looks


@pytest.mark.parametrize("look", [racecar_skin_2d, racecar_skin_3d, urdf_skin])
def test_every_look_draws_something_on_the_urdf_frames(look):
    """A skin is geometry keyed to the frames the placement returns."""
    car = UdeSRacecarDyn3D()

    geometry = look(car)

    assert geometry
    assert sum(len(prims) for prims in geometry.values()) >= 10
    frames = set(racecar_frames(0.0, 0.0, 0.0, 0.0)) | {"world"}
    assert set(geometry).issubset(frames)


@pytest.mark.parametrize("look", [racecar_skin_2d, racecar_skin_3d, urdf_skin])
def test_the_plant_places_every_frame_its_look_needs(look):
    """The animator flattens the look against ``tf``: a missing frame is a KeyError."""
    car = UdeSRacecarDyn3D()
    car.scene_bounds = (-2.0, 2.0, -1.0, 1.0)

    geometry = look(car)
    frames = car.tf(np.zeros(car.n), np.zeros(car.m))

    assert set(geometry) - {"world"} <= set(frames)
    assert "world" not in frames  # the animator holds the world frame itself


def test_the_scene_is_optional_and_asked_for_by_the_plant():
    """No ``scene_bounds``, no ground: a look never invents a world around a plant."""
    bare = UdeSRacecarDyn3D()
    grounded = UdeSRacecarDyn3D()
    grounded.scene_bounds = (-2.0, 2.0, -1.0, 1.0)

    assert "world" not in racecar_skin_3d(bare)
    assert "world" not in racecar_skin_2d(bare)
    assert racecar_skin_3d(grounded)["world"]
    assert racecar_skin_2d(grounded)["world"]


@pytest.mark.parametrize("scene", [track_scene, track_lines])
def test_the_walls_of_the_scene_are_optional(scene):
    """The same ground with and without the corridor walls, and never camera-fitted."""
    bounds = (-1.0, 3.0, -1.0, 1.0)

    open_floor = scene(bounds, walls=False)
    corridor = scene(bounds, walls=True)

    assert len(corridor) == len(open_floor) + 2
    assert all(not primitive.camera_fit for primitive in corridor)


def test_the_scene_paints_a_circuit_as_a_closed_line():
    """A loop sampled without its first point repeated is closed again for drawing."""
    loop = circuit_waypoints(4.0, 3.0, 1.0)

    painted = track_scene((-3.0, 3.0, -3.0, 3.0), path=loop)[-1]

    points = np.asarray(painted.points_at(0.0))
    assert len(points) == len(loop) + 1
    assert points[-1, :2] == pytest.approx(loop[0])
    assert points[:, 2] == pytest.approx(np.zeros(len(points)), abs=1e-2)


def test_the_scene_leaves_an_open_path_open():
    """A lane change is not a lap: no phantom segment back to the start."""
    lane_change = np.array([[0.0, 0.0], [5.0, 0.0], [10.0, 1.0]])

    painted = track_scene((-1.0, 11.0, -2.0, 2.0), path=lane_change)[-1]

    points = np.asarray(painted.points_at(0.0))
    assert len(points) == len(lane_change)
    assert points[-1, :2] == pytest.approx(lane_change[-1])


# The plant that drives the looks


@pytest.mark.parametrize(
    "car_factory, u",
    [
        (UdeSRacecar, np.array([1.0, 0.2])),
        (UdeSRacecarDyn, np.array([40.0, 0.2])),
        (UdeSRacecarDyn3D, np.array([40.0, 0.2])),
    ],
)
def test_the_3d_skin_attaches_to_both_plants(car_factory, u):
    """Kinematic and dynamic ``tf`` publish every frame ``racecar_skin_3d`` draws."""
    car = car_factory()
    car.skin = racecar_skin_3d

    geometry = car.skin(car)
    frames = car.tf(np.zeros(car.n), u)

    assert set(geometry) - {"world"} <= set(frames)
    assert set(URDF_LINKS).issubset(frames)


def test_the_3d_plant_adds_only_the_wheel_angles():
    """Eleven states: the nine of the car, plus one rolling angle per axle."""
    car = UdeSRacecarDyn3D()
    plain = UdeSRacecarDyn()
    x = np.array([1.0, 0.4, 0.2, 4.0, -0.3, 0.8, 85.0, 0.1, 30.0, 2.0, 3.0])
    u = np.array([40.0, 0.2])

    dx = car.f(x, u)

    assert car.n == 11
    assert dx[:9] == pytest.approx(plain.f(x[:9], u))
    assert dx[9] == pytest.approx(x[6])
    assert dx[10] > 0.0


def test_the_3d_plant_places_its_own_skin():
    """``tf`` feeds the URDF frames from the state, wheel angles included."""
    car = UdeSRacecarDyn3D()
    x = np.zeros(car.n)
    x[0], x[1], x[2], x[7], x[9] = 2.0, 1.0, 0.5, 0.2, 1.3

    frames = car.tf(x, np.zeros(car.m))

    assert set(URDF_LINKS).issubset(frames)
    assert np.asarray(frames["body"])[:2, 3] == pytest.approx([2.0, 1.0])
    rolled = np.asarray(frames["left_rear_wheel"])
    assert rolled[:3, :3] @ rolled[:3, :3].T == pytest.approx(np.eye(3), abs=1e-9)
