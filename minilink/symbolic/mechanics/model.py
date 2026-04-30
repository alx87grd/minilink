"""
Mechanical model before equations of motion — frames, bodies, DH chains, loads.

Call :meth:`derive` to obtain a symbolic :class:`~minilink.symbolic.mechanics.symbolic_system.MechanicalSystem`.
"""

import sympy as sp
from sympy.physics.mechanics import (
    Particle,
    Point,
    ReferenceFrame,
    RigidBody,
    dynamicsymbols,
)
from sympy.physics.mechanics import (
    inertia as make_inertia,
)
from sympy.physics.vector import time_derivative


class MechanicalModel:
    """
    Builder for symbolic multibody systems (pre-EoM).

    Typical workflow
    ----------------
    1. Create a :class:`MechanicalModel` and declare parameters / coordinates.
    2. Add bodies (DH chains and/or manual frames).
    3. Add gravity, springs, dampers, etc.
    4. Call ``derive()`` to obtain a symbolic ``MechanicalSystem`` (SymPy).
    5. Call ``sys.to_minilink(params)`` for a numeric plant (NumPy or JAX; see :meth:`~minilink.symbolic.mechanics.symbolic_system.MechanicalSystem.to_minilink`).
    """

    def __init__(self, name="System"):
        self.name = name

        # Newtonian (inertial) reference frame and fixed origin
        self.N = ReferenceFrame("N")
        self.O = Point("O")
        self.O.set_vel(self.N, 0)

        # Symbolic quantities
        self._params = {}
        self._coordinates = []
        self._speeds = []
        self._kdes = []

        # Mechanical entities
        self._bodies = []
        self._frames = {"N": self.N}
        self._points = {"O": self.O}

        # Forces / torques (explicit loads for Kane)
        self._forces = []
        self._torques = []
        self._gravity_vec = None

        # Springs and dampers (stored as definitions, computed at derivation)
        self._springs = []
        self._dampers = []

        # Actuator bookkeeping
        self._B_matrix = None

        # DH chain state
        self._dh_links = []
        self._effector_point = None
        self._chain_points = []

    # Variable management
    def parameters(self, names, **assumptions):
        """
        Create symbolic constants (masses, lengths, gravity ...).

        Returns a single Symbol or a tuple of Symbols.
        """
        if "positive" not in assumptions and "real" not in assumptions:
            assumptions["positive"] = True

        syms = sp.symbols(names, **assumptions)
        if isinstance(syms, sp.Symbol):
            syms = (syms,)
        for s in syms:
            self._params[str(s)] = s
        return syms if len(syms) > 1 else syms[0]

    def coordinates(self, names):
        """
        Create generalized coordinates ``q(t)``.

        Matching generalized speeds and kinematic differential equations
        are created automatically (needed by Kane's method).
        """
        t = sp.Symbol("t")
        name_list = names.split()
        coords = []
        for n in name_list:
            q = dynamicsymbols(n)
            self._coordinates.append(q)

            u_name = n.replace("q", "dq") if "q" in n else f"d{n}"
            u = dynamicsymbols(u_name)
            self._speeds.append(u)
            self._kdes.append(q.diff(t) - u)

            coords.append(q)
        return tuple(coords) if len(coords) > 1 else coords[0]

    def speeds(self, names):
        """Override auto-created speeds with custom definitions."""
        t = sp.Symbol("t")
        name_list = names.split()
        out = []
        for i, n in enumerate(name_list):
            u = dynamicsymbols(n)
            if i < len(self._speeds):
                self._speeds[i] = u
                self._kdes[i] = self._coordinates[i].diff(t) - u
            else:
                self._speeds.append(u)
            out.append(u)
        return tuple(out) if len(out) > 1 else out[0]

    @property
    def q(self):
        """Generalized coordinates list."""
        return list(self._coordinates)

    @property
    def dq(self):
        """Time-derivatives of generalized coordinates."""
        t = sp.Symbol("t")
        return [qi.diff(t) for qi in self._coordinates]

    @property
    def u(self):
        """Generalized speeds list."""
        return list(self._speeds)

    @property
    def dof(self):
        """Degrees of freedom (number of coordinates)."""
        return len(self._coordinates)

    # Frame / point helpers
    def add_frame(self, name, parent=None, rotation_type="axis", **kwargs):
        """
        Add a reference frame oriented relative to *parent*.

        Parameters
        ----------
        rotation_type : ``'axis'`` | ``'body'`` | ``'dcm'``
            ``'axis'``  -- kwargs: *axis* (Vector), *angle* (Expr)
            ``'body'``  -- kwargs: *angles* (tuple), *rotation_order* (str)
            ``'dcm'``   -- kwargs: *dcm* (Matrix)
        """
        parent = parent or self.N
        frame = ReferenceFrame(name)

        if rotation_type == "axis":
            frame.orient_axis(parent, kwargs["axis"], kwargs["angle"])
        elif rotation_type == "body":
            frame.orient_body_fixed(parent, kwargs["angles"], kwargs["rotation_order"])
        elif rotation_type == "dcm":
            frame.orient_explicit(parent, kwargs["dcm"])

        self._frames[name] = frame
        return frame

    def add_point(self, name, parent_point=None, position=None):
        """Add a point with position relative to *parent_point*."""
        parent_point = parent_point or self.O
        pt = Point(name)
        pt.set_pos(parent_point, position if position is not None else sp.S(0))
        self._points[name] = pt
        return pt

    # Bodies
    def add_body(self, name, mass, inertia=None, frame=None, com=None):
        """
        Add a rigid body.

        If *frame* / *com* are ``None`` they are created automatically.
        *inertia* is a SymPy Dyadic about the COM; ``None`` -> point-mass.
        """
        if frame is None:
            frame = self._frames.get(name) or ReferenceFrame(name)
            self._frames[name] = frame
        if com is None:
            com = Point(f"{name}_cm")
            self._points[f"{name}_cm"] = com
        if inertia is None:
            inertia = make_inertia(frame, 0, 0, 0)

        body = RigidBody(name, com, frame, mass, (inertia, com))
        self._bodies.append(body)
        return body

    def add_particle(self, name, mass, point=None):
        """Add a point mass."""
        if point is None:
            point = Point(f"{name}_pt")
            self._points[f"{name}_pt"] = point
        particle = Particle(name, point, mass)
        self._bodies.append(particle)
        return particle

    # Denavit-Hartenberg chain builder
    def add_dh_chain(self, dh_table, link_properties):
        """
        Build a serial kinematic chain from a DH parameter table.

        Parameters
        ----------
        dh_table : list[dict]
            Each entry has keys ``theta``, ``d``, ``a``, ``alpha``.
        link_properties : list[dict]
            Keys: ``mass``, ``inertia`` (dict or Dyadic), ``com_offset``.
        """
        parent_frame = self.N
        parent_origin = self.O

        for i, (dh, props) in enumerate(zip(dh_table, link_properties)):
            ln = f"link{i + 1}"
            theta, d, a, alpha = dh["theta"], dh["d"], dh["a"], dh["alpha"]

            if alpha == 0:
                link_frame = ReferenceFrame(ln)
                link_frame.orient_axis(parent_frame, parent_frame.z, theta)
                inter = link_frame
            else:
                inter = ReferenceFrame(f"{ln}_inter")
                inter.orient_axis(parent_frame, parent_frame.z, theta)
                link_frame = ReferenceFrame(ln)
                link_frame.orient_axis(inter, inter.x, alpha)
            self._frames[ln] = link_frame

            joint_pt = Point(f"{ln}_joint")
            joint_pt.set_pos(parent_origin, d * parent_frame.z + a * inter.x)
            self._points[f"{ln}_joint"] = joint_pt

            com = Point(f"{ln}_cm")
            com_offset = props.get("com_offset", a / 2 if a != 0 else 0)

            if isinstance(com_offset, dict):
                cx = com_offset.get("x", 0)
                cy = com_offset.get("y", 0)
                cz = com_offset.get("z", 0)
                com.set_pos(
                    joint_pt,
                    cx * link_frame.x + cy * link_frame.y + cz * link_frame.z,
                )
            elif hasattr(com_offset, "dot"):
                com.set_pos(joint_pt, com_offset)
            else:
                com.set_pos(parent_origin, d * parent_frame.z + com_offset * inter.x)
            self._points[f"{ln}_cm"] = com

            mass = props["mass"]
            inertia_spec = props.get("inertia", None)
            if inertia_spec is None or inertia_spec == 0:
                inertia_val = make_inertia(link_frame, 0, 0, 0)
            elif isinstance(inertia_spec, dict):
                inertia_val = make_inertia(
                    link_frame,
                    inertia_spec.get("Ixx", 0),
                    inertia_spec.get("Iyy", 0),
                    inertia_spec.get("Izz", 0),
                    inertia_spec.get("Ixy", 0),
                    inertia_spec.get("Iyz", 0),
                    inertia_spec.get("Izx", 0),
                )
            else:
                inertia_val = inertia_spec

            body = RigidBody(ln, com, link_frame, mass, (inertia_val, com))
            self._bodies.append(body)

            self._dh_links.append(
                {
                    "frame": link_frame,
                    "inter": inter,
                    "origin": joint_pt,
                    "body": body,
                    "com": com,
                    "dh": dh,
                }
            )

            parent_frame = link_frame
            parent_origin = joint_pt

        if self._dh_links:
            self._effector_point = parent_origin
            self._chain_points = [self.O] + [lk["origin"] for lk in self._dh_links]

    def set_effector(self, point):
        """Declare *point* as the end-effector (used by FK / Jacobian)."""
        self._effector_point = point

    def set_kinematic_chain(self, points):
        """
        Declare the ordered list of points forming the kinematic chain
        (used for animation / chain forward kinematics).

        For DH chains this is set automatically.
        """
        self._chain_points = list(points)

    # Forces & torques
    def add_gravity(self, accel_vector):
        """Set gravitational acceleration.  Example: ``-g * model.N.y``."""
        self._gravity_vec = accel_vector

    def add_force(self, point, force_vector):
        """Apply an external force at *point*."""
        self._forces.append((point, force_vector))

    def add_torque(self, frame, torque_vector):
        """Apply an external torque on *frame*."""
        self._torques.append((frame, torque_vector))

    def add_spring(self, point_a, point_b, stiffness, natural_length=0):
        """
        Linear spring between two points.

        Contributes to conservative forces ``g(q)`` via potential energy.
        """
        self._springs.append((point_a, point_b, stiffness, natural_length))

    def add_damper(self, point_a, point_b, damping):
        """
        Linear viscous damper between two points.

        Contributes to dissipative forces ``d(q, dq)`` via Rayleigh
        dissipation: ``R = 1/2 * b * |v_rel|^2``.
        """
        self._dampers.append((point_a, point_b, damping))

    def set_input_matrix(self, B):
        """Provide the actuator matrix B(q) directly as a SymPy Matrix."""
        self._B_matrix = B

    # Forward kinematics
    def forward_kinematics(self, point=None, simplify=True):
        """
        Symbolic position of *point* (default: end-effector) in the N frame.

        Returns a SymPy column Matrix ``[x, y, z]``.
        """
        point = point or self._effector_point
        if point is None:
            raise ValueError("No point specified and no end-effector set.")
        pos = point.pos_from(self.O)
        fk = sp.Matrix([pos.dot(self.N.x), pos.dot(self.N.y), pos.dot(self.N.z)])
        if simplify:
            fk = fk.applyfunc(sp.trigsimp)
        return fk

    def jacobian(self, point=None):
        """
        Geometric (translational) Jacobian of *point* w.r.t. coordinates.

        Always-zero rows (e.g. z for planar systems) are stripped.
        """
        fk = self.forward_kinematics(point)
        active = [
            i
            for i in range(fk.rows)
            if fk[i] != 0 or any(fk[i].diff(qi) != 0 for qi in self._coordinates)
        ]
        if active:
            fk = sp.Matrix([fk[i] for i in active])
        return fk.jacobian(self._coordinates)

    def _compute_chain_fk(self, simplify=True):
        """Return list of (3,1) position matrices for each chain point."""
        if not self._chain_points:
            return None
        result = []
        for pt in self._chain_points:
            pos = pt.pos_from(self.O)
            fk = sp.Matrix(
                [
                    pos.dot(self.N.x),
                    pos.dot(self.N.y),
                    pos.dot(self.N.z),
                ]
            )
            if simplify:
                fk = fk.applyfunc(sp.trigsimp)
            result.append(fk)
        return result

    # Internal helpers (used by derivation)
    def _setup_velocities(self):
        """Set velocities of every body in the Newtonian frame."""
        for body in self._bodies:
            com = body.masscenter if isinstance(body, RigidBody) else body.point
            pos = com.pos_from(self.O)
            vel = time_derivative(pos, self.N)
            com.set_vel(self.N, vel)

    def _compute_kinetic_energy(self):
        T = sp.S(0)
        for body in self._bodies:
            T += body.kinetic_energy(self.N)
        return T

    def _compute_potential_energy(self):
        V = sp.S(0)
        # Gravity
        if self._gravity_vec is not None:
            for body in self._bodies:
                mass = body.mass
                pos = (
                    body.masscenter.pos_from(self.O)
                    if isinstance(body, RigidBody)
                    else body.point.pos_from(self.O)
                )
                V -= mass * self._gravity_vec.dot(pos)
        # Springs
        for pt_a, pt_b, k, L0 in self._springs:
            disp = pt_b.pos_from(pt_a)
            if L0 == 0:
                V += sp.Rational(1, 2) * k * disp.dot(disp)
            else:
                dist = disp.magnitude()
                V += sp.Rational(1, 2) * k * (dist - L0) ** 2
        return V

    def _compute_dissipative_forces(self, simplify=True):
        """
        Compute symbolic d(q, dq) from Rayleigh dissipation.

        R = sum_dampers 1/2 * b * |v_rel|^2
        d_i = dR / dq_dot_i
        """
        t = sp.Symbol("t")
        dof = self.dof
        qdots = [qi.diff(t) for qi in self._coordinates]

        if not self._dampers:
            return sp.zeros(dof, 1)

        R = sp.S(0)
        for pt_a, pt_b, b in self._dampers:
            vel_rel = pt_b.vel(self.N) - pt_a.vel(self.N)
            R += sp.Rational(1, 2) * b * vel_rel.dot(vel_rel)

        d = sp.zeros(dof, 1)
        for i in range(dof):
            di = sp.diff(R, qdots[i])
            if simplify:
                di = sp.trigsimp(di)
            d[i] = di
        return d

    def _collect_loads(self):
        """Return (force_list, torque_list) for Kane's method."""
        forces = list(self._forces)
        torques = list(self._torques)
        if self._gravity_vec is not None:
            for body in self._bodies:
                if isinstance(body, RigidBody):
                    forces.append((body.masscenter, body.mass * self._gravity_vec))
                else:
                    forces.append((body.point, body.mass * self._gravity_vec))
        # Spring forces
        for pt_a, pt_b, k, L0 in self._springs:
            disp = pt_b.pos_from(pt_a)
            dist = disp.magnitude()
            direction = disp.normalize()
            force_mag = k * (dist - L0)
            forces.append((pt_a, force_mag * direction))
            forces.append((pt_b, -force_mag * direction))
        # Damper forces
        for pt_a, pt_b, b in self._dampers:
            vel_rel = pt_b.vel(self.N) - pt_a.vel(self.N)
            forces.append((pt_a, b * vel_rel))
            forces.append((pt_b, -b * vel_rel))
        return forces, torques

    # EOM derivation
    def derive(self, method="lagrange", simplify=True):
        """
        Derive equations of motion.

        Parameters
        ----------
        method : ``'lagrange'`` | ``'kane'``
        simplify : bool
            Apply trigsimp to resulting matrices.

        Returns
        -------
        MechanicalSystem
        """
        from .derivation import derive_kane, derive_lagrange

        self._setup_velocities()

        if method == "lagrange":
            return derive_lagrange(self, simplify=simplify)
        elif method == "kane":
            return derive_kane(self, simplify=simplify)
        else:
            raise ValueError(f"Unknown method '{method}'. Use 'lagrange' or 'kane'.")
