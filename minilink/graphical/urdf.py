"""A small URDF reader: xacro expansion, link / joint tables and forward kinematics.

Enough of xacro for the racecar description (no ROS needed): ``xacro:arg``
defaults, ``$(arg ...)``, ``$(find <package>)``, ``xacro:include``,
``xacro:property``, parameterised ``xacro:macro`` calls and ``${...}`` arithmetic.
On links, joints, visuals and materials, the expansion matches what
``ros2 run xacro xacro racecar.xacro`` produces from the same files.

The parsed URDF is a plain dict (links, joints, materials) and
:func:`forward_kinematics` composes the joint transforms
``T_child = T_parent @ T_origin(xyz, rpy) @ T_joint(axis, q)`` with the ``xp``
idiom, so it runs inside a minilink ``tf`` on NumPy or JAX.
"""

from __future__ import annotations

import copy
import math
import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE3, Rx, Ry, Rz

XACRO_NS = "http://www.ros.org/wiki/xacro"
_X = "{" + XACRO_NS + "}"

# xacro


def expand_xacro(path, args=None, packages=None):
    """Expand a xacro file into a plain URDF ``<robot>`` element.

    Parameters
    ----------
    path : str or Path
        Top-level ``.xacro`` file.
    args : dict, optional
        Values for ``xacro:arg`` (defaults come from the file).
    packages : dict, optional
        ``{package_name: directory}`` used for ``$(find package_name)``. Missing
        packages resolve to the parent of the file's directory.

    Returns
    -------
    xml.etree.ElementTree.Element
        The expanded ``<robot>`` element (URDF, no xacro tags left).
    """
    path = Path(path).resolve()
    packages = dict(packages or {})
    context = {
        "args": dict(args or {}),
        "props": {},
        "macros": {},
        "packages": packages,
    }
    context["default_package_dir"] = path.parent.parent
    root = ET.parse(path).getroot()
    robot = ET.Element(
        "robot", {"name": _substitute(root.get("name", "robot"), context, {})}
    )
    _expand_children(root, robot, context, {}, path.parent)
    return robot


def _expand_children(source, target, context, scope, base_dir):
    for node in list(source):
        if not isinstance(node.tag, str):
            continue
        if node.tag.startswith(_X):
            _expand_xacro_tag(node, target, context, scope, base_dir)
            continue
        element = ET.SubElement(
            target,
            node.tag,
            {k: _substitute(v, context, scope) for k, v in node.attrib.items()},
        )
        if node.text and node.text.strip():
            element.text = _substitute(node.text, context, scope)
        _expand_children(node, element, context, scope, base_dir)


def _expand_xacro_tag(node, target, context, scope, base_dir):
    kind = node.tag[len(_X) :]
    if kind == "arg":
        context["args"].setdefault(
            node.get("name"), _substitute(node.get("default", ""), context, scope)
        )
    elif kind == "property":
        context["props"][node.get("name")] = _substitute(
            node.get("value", ""), context, scope
        )
    elif kind == "include":
        filename = Path(_substitute(node.get("filename"), context, scope))
        if not filename.is_absolute():
            filename = base_dir / filename
        if filename.exists():
            _expand_children(
                ET.parse(filename).getroot(), target, context, scope, filename.parent
            )
    elif kind == "macro":
        params = []
        for token in node.get("params", "").split():
            name, _, default = token.partition(":=")
            params.append((name.lstrip("*"), default or None))
        context["macros"][node.get("name")] = (params, node)
    elif kind in ("if", "unless"):
        value = _substitute(node.get("value", ""), context, scope).strip().lower()
        if (value in ("1", "true")) == (kind == "if"):
            _expand_children(node, target, context, scope, base_dir)
    elif kind in context["macros"]:
        params, body = context["macros"][kind]
        local = dict(scope)
        for name, default in params:
            value = node.get(name, default)
            local[name] = (
                _substitute(value, context, scope) if value is not None else ""
            )
        _expand_children(body, target, context, local, base_dir)
    else:
        raise ValueError(f"unsupported xacro tag: xacro:{kind}")


def _substitute(text, context, scope):
    """Resolve ``$(arg x)``, ``$(find pkg)`` and ``${expression}`` in *text*."""
    if text is None:
        return None

    def dollar_paren(match):
        command, _, value = match.group(1).partition(" ")
        value = value.strip()
        if command == "arg":
            return str(context["args"].get(value, ""))
        if command == "find":
            return str(context["packages"].get(value, context["default_package_dir"]))
        if command in ("env", "optenv"):
            name, _, default = value.partition(" ")
            return os.environ.get(name, default)
        raise ValueError(f"unsupported substitution $({command})")

    def dollar_brace(match):
        names = {**context["props"], **scope}
        values = {}
        for name, value in names.items():
            try:
                values[name] = float(value)
            except (TypeError, ValueError):
                values[name] = value
        result = eval(
            match.group(1), {"__builtins__": {}, "pi": math.pi, "math": math}, values
        )
        return f"{result:g}" if isinstance(result, float) else str(result)

    text = re.sub(r"\$\(([^)]*)\)", dollar_paren, text)
    return re.sub(r"\$\{([^}]*)\}", dollar_brace, text)


# URDF tables


def parse_urdf(robot, strip_prefix=True):
    """Read links, joints and materials of an expanded URDF ``<robot>``.

    Returns
    -------
    dict
        ``{"name", "materials": {name: rgba}, "links": {name: {"visuals": [...]}},
        "joints": {name: {...}}, "root": link}``. A visual is
        ``{"xyz", "rpy", "geometry": {"type", ...}, "material": rgba or None}``;
        a joint is ``{"type", "parent", "child", "xyz", "rpy", "axis", "limit"}``.
        With *strip_prefix*, the ``"<prefix>/"`` namespace is removed from link names.
    """
    if isinstance(robot, (str, Path)):
        robot = ET.parse(robot).getroot()

    def link_name(name):
        return name.split("/")[-1] if strip_prefix else name

    def vector(element, attribute, default):
        if element is None or element.get(attribute) is None:
            return np.array(default, dtype=float)
        return np.array(element.get(attribute).split(), dtype=float)

    materials = {}
    for material in robot.findall("material"):
        color = material.find("color")
        if color is not None:
            materials[material.get("name")] = vector(
                color, "rgba", [0.5, 0.5, 0.5, 1.0]
            )

    def material_of(visual):
        # as the ROS parser: a <material> misplaced inside <geometry> (the racecar's
        # hokuyo) is ignored, and the mesh file's own colours apply
        element = visual.find("material")
        if element is None:
            return None
        color = element.find("color")
        if color is not None:
            return vector(color, "rgba", [0.5, 0.5, 0.5, 1.0])
        return materials.get(element.get("name"))

    links = {}
    for link in robot.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            shape = next(iter(visual.find("geometry")))
            geometry = {"type": shape.tag}
            if shape.tag == "mesh":
                geometry["filename"] = shape.get("filename")
                geometry["scale"] = vector(shape, "scale", [1.0, 1.0, 1.0])
            elif shape.tag == "box":
                geometry["size"] = vector(shape, "size", [0.0, 0.0, 0.0])
            elif shape.tag == "cylinder":
                geometry["radius"] = float(shape.get("radius"))
                geometry["length"] = float(shape.get("length"))
            elif shape.tag == "sphere":
                geometry["radius"] = float(shape.get("radius"))
            visuals.append(
                {
                    "xyz": vector(origin, "xyz", [0.0, 0.0, 0.0]),
                    "rpy": vector(origin, "rpy", [0.0, 0.0, 0.0]),
                    "geometry": geometry,
                    "material": material_of(visual),
                }
            )
        links[link_name(link.get("name"))] = {"visuals": visuals}

    joints = {}
    for joint in robot.findall("joint"):
        origin = joint.find("origin")
        limit = joint.find("limit")
        joints[joint.get("name")] = {
            "type": joint.get("type"),
            "parent": link_name(joint.find("parent").get("link")),
            "child": link_name(joint.find("child").get("link")),
            "xyz": vector(origin, "xyz", [0.0, 0.0, 0.0]),
            "rpy": vector(origin, "rpy", [0.0, 0.0, 0.0]),
            "axis": vector(joint.find("axis"), "xyz", [1.0, 0.0, 0.0]),
            "limit": None
            if limit is None
            else (float(limit.get("lower", "nan")), float(limit.get("upper", "nan"))),
        }

    children = {j["child"] for j in joints.values()}
    roots = [name for name in links if name not in children]
    return {
        "name": robot.get("name"),
        "materials": materials,
        "links": links,
        "joints": joints,
        "root": roots[0] if roots else None,
    }


def load_urdf(path, args=None, packages=None):
    """Expand (if xacro) and parse a URDF file; see :func:`parse_urdf`."""
    path = Path(path)
    if path.suffix == ".xacro":
        robot = expand_xacro(path, args=args, packages=packages)
    else:
        robot = ET.parse(path).getroot()
    urdf = parse_urdf(robot)
    urdf["xml"] = robot
    return urdf


def write_urdf(urdf_or_robot, path, mesh_prefix=None):
    """Write an expanded ``<robot>`` element (or a parsed URDF with ``"xml"``) to *path*.

    Expansion resolves ``$(find <package>)`` against the filesystem, so the mesh
    filenames of the element are absolute. *mesh_prefix* rewrites each of them as
    ``mesh_prefix + basename`` — pass ``"package://<package>/meshes/"`` to write a file
    that is portable (and that gives away nothing about the machine that wrote it).
    """
    robot = urdf_or_robot["xml"] if isinstance(urdf_or_robot, dict) else urdf_or_robot
    robot = copy.deepcopy(robot)
    if mesh_prefix is not None:
        for mesh in robot.iter("mesh"):
            mesh.set("filename", mesh_prefix + Path(mesh.get("filename")).name)
    ET.indent(robot, space="  ")
    Path(path).write_text(
        '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="unicode") + "\n"
    )


# Kinematics


def rpy_matrix(rpy):
    """URDF fixed-axis roll-pitch-yaw: ``R = Rz(yaw) Ry(pitch) Rx(roll)``."""
    roll, pitch, yaw = (float(a) for a in rpy)
    return np.asarray(Rz(yaw) @ Ry(pitch) @ Rx(roll))


def origin_transform(xyz, rpy):
    """4x4 pose of a URDF ``<origin xyz rpy>``."""
    return np.asarray(SE3(rpy_matrix(rpy), np.asarray(xyz, dtype=float)))


def axis_rotation(axis, q):
    """4x4 rotation by *q* about the unit *axis* (Rodrigues; *q* may be a JAX tracer)."""
    xp = array_module(q)
    k = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    R = np.eye(3) + xp.sin(q) * K + (1.0 - xp.cos(q)) * (K @ K)
    return SE3(R, 0.0)


def forward_kinematics(urdf, q=None, T_root=None, origins=None):
    """World pose of every link for joint values *q*.

    Parameters
    ----------
    urdf : dict
        Parsed URDF (:func:`parse_urdf`).
    q : dict, optional
        ``{joint_name: angle [rad] or displacement [m]}``; missing joints are 0.
    T_root : (4, 4) array, optional
        World pose of the root link (identity by default).
    origins : dict, optional
        ``{joint_name: xyz}`` replacing joint origin positions (e.g. a wheelbase).

    Returns
    -------
    dict[str, (4, 4) array]
        ``{link_name: T_world_link}``.
    """
    q = q or {}
    origins = origins or {}
    xp = array_module(*(v for v in q.values()), *([] if T_root is None else [T_root]))
    T_root = xp.eye(4) if T_root is None else T_root
    by_parent = {}
    for name, joint in urdf["joints"].items():
        by_parent.setdefault(joint["parent"], []).append((name, joint))

    frames = {urdf["root"]: T_root}
    stack = [urdf["root"]]
    while stack:
        parent = stack.pop()
        for name, joint in by_parent.get(parent, []):
            if name in origins:
                T = frames[parent] @ SE3(rpy_matrix(joint["rpy"]), origins[name])
            else:
                T = frames[parent] @ origin_transform(joint["xyz"], joint["rpy"])
            value = q.get(name, 0.0)
            if joint["type"] in ("revolute", "continuous"):
                T = T @ axis_rotation(joint["axis"], value)
            elif joint["type"] == "prismatic":
                axis = np.asarray(joint["axis"], dtype=float)
                T = T @ SE3(np.eye(3), value * axis)
            frames[joint["child"]] = T
            stack.append(joint["child"])
    return frames


def resolve_mesh_path(filename, mesh_dir):
    """Map a URDF mesh URI (``file://.../meshes/x.STL``, ``package://``) to *mesh_dir*."""
    return Path(mesh_dir) / Path(re.sub(r"^(file|package)://", "", filename)).name
