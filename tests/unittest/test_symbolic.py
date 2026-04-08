"""Tests for minilink.mechanics.symbolic (requires SymPy)."""

import unittest

import numpy as np

try:
    import sympy as sp
    from sympy.physics.mechanics import dynamicsymbols

    from minilink.mechanics.symbolic import MechanicalModel, derive_lagrange
    from minilink.mechanics.symbolic.export import create_minilink_system
    from minilink.mechanics.symbolic.symbolic_system import MechanicalSystem

    HAS_SYMPY = True
except ImportError:
    HAS_SYMPY = False

try:
    import jax  # noqa: F401

    HAS_JAX = True
except ImportError:
    HAS_JAX = False


@unittest.skipUnless(HAS_SYMPY, "sympy not installed")
class TestSymbolicMechanics(unittest.TestCase):
    def test_single_link_pendulum_derive_and_export(self):
        m = MechanicalModel("Pendulum")
        g_sym, mass, length = m.parameters("g m l")
        q1 = m.coordinates("q1")
        m.add_dh_chain(
            [{"theta": q1, "d": 0, "a": length, "alpha": 0}],
            [{"mass": mass, "inertia": {"Izz": 0}, "com_offset": length / 2}],
        )
        m.add_gravity(-g_sym * m.N.y)
        m._setup_velocities()
        sys = derive_lagrange(m, simplify=True)
        self.assertEqual(sys.dof, 1)
        self.assertIsNotNone(sys.H)
        self.assertIsNotNone(sys.chain_fk)

        params = {g_sym: 9.81, mass: 1.0, length: 1.0}
        num = sys.to_minilink(parameters=params)
        x = np.array([0.1, 0.0])
        u = np.zeros(1)
        dx = num.f(x, u, 0.0)
        self.assertEqual(dx.shape, (2,))
        self.assertAlmostEqual(dx[0], 0.0, places=9)
        # Composite pendulum (mass + link inertia), not point-mass at pivot
        self.assertTrue(dx[1] < 0)
        self.assertTrue(np.sign(dx[1]) == -np.sign(np.sin(0.1)))

    def test_hand_built_symbolic_system_to_minilink(self):
        """1-DOF SymPy matrices only (minimal dummy model)."""
        q = dynamicsymbols("q1")

        class _DummyModel:
            name = "ScalarTest"
            dof = 1
            _params = {}

            @property
            def q(self):
                return [q]

        sys = MechanicalSystem(_DummyModel())
        sys.H = sp.Matrix([[sp.Symbol("m")]])
        sys.C = sp.zeros(1)
        sys.g = sp.Matrix([[sp.Symbol("k") * q]])
        sys.d = sp.zeros(1, 1)
        sys.B = sp.eye(1)
        sys.method = "manual"

        num = create_minilink_system(
            sys, parameters={sp.Symbol("m"): 2.0, sp.Symbol("k"): 0.0}
        )
        x = np.array([0.0, 1.0])
        u = np.array([0.0])
        dx = num.f(x, u, 0.0)
        np.testing.assert_allclose(dx, np.array([1.0, 0.0]), rtol=0, atol=1e-9)

    def test_kinematic_geometry_matches_transforms(self):
        m = MechanicalModel("Pendulum")
        g_sym, mass, length = m.parameters("g m l")
        q1 = m.coordinates("q1")
        m.add_dh_chain(
            [{"theta": q1, "d": 0, "a": length, "alpha": 0}],
            [{"mass": mass, "inertia": {"Izz": 0}, "com_offset": length / 2}],
        )
        m.add_gravity(-g_sym * m.N.y)
        m._setup_velocities()
        sys = derive_lagrange(m, simplify=True)
        num = sys.to_minilink({g_sym: 9.81, mass: 1.0, length: 0.5})
        prim = num.get_kinematic_geometry()
        T = num.get_kinematic_transforms(np.zeros(2), np.zeros(1), 0.0)
        self.assertEqual(len(prim), len(T))

    @unittest.skipUnless(HAS_SYMPY and HAS_JAX, "sympy and jax required")
    def test_to_minilink_jax_returns_jax_mechanical_system(self):
        from minilink.mechanics.mechanical import JaxMechanicalSystem

        m = MechanicalModel("Pendulum")
        g_sym, mass, length = m.parameters("g m l")
        q1 = m.coordinates("q1")
        m.add_dh_chain(
            [{"theta": q1, "d": 0, "a": length, "alpha": 0}],
            [{"mass": mass, "inertia": {"Izz": 0}, "com_offset": length / 2}],
        )
        m.add_gravity(-g_sym * m.N.y)
        m._setup_velocities()
        sys = derive_lagrange(m, simplify=True)
        num = sys.to_minilink({g_sym: 9.81, mass: 1.0, length: 0.5}, backend="jax")
        self.assertIsInstance(num, JaxMechanicalSystem)


if __name__ == "__main__":
    unittest.main()
