import unittest

import numpy as np
import pytest

from minilink.core.blocks.basic import Integrator, PropController
from minilink.core.blocks.sources import Source, Step


class TestBlocks(unittest.TestCase):
    def test_source(self):
        s = Source(p=2)
        s.params["value"] = np.array([3.0, 4.0])
        y = s.h(x=[], u=[], t=0)
        np.testing.assert_array_equal(y, np.array([3.0, 4.0]))

    def test_step_source(self):
        step = Step(
            initial_value=np.array([0.0]), final_value=np.array([1.0]), step_time=2.0
        )
        y_before = step.h(x=[], u=[], t=1.0)
        y_after = step.h(x=[], u=[], t=3.0)
        np.testing.assert_array_equal(y_before, np.array([0.0]))
        np.testing.assert_array_equal(y_after, np.array([1.0]))

    def test_integrator_dynamics_and_output(self):
        plant = Integrator()
        plant.params["k"] = 2.0

        np.testing.assert_array_equal(
            plant.f(np.array([3.0]), np.array([4.0])),
            np.array([8.0]),
        )
        np.testing.assert_array_equal(
            plant.h(np.array([3.0]), np.array([4.0])),
            np.array([3.0]),
        )

    def test_prop_controller_scales_tracking_error(self):
        controller = PropController()
        controller.params["Kp"] = 2.5

        np.testing.assert_array_equal(
            controller.ctl(np.array([]), np.array([3.0, 1.0])),
            np.array([5.0]),
        )

    def test_basic_blocks_are_jax_jittable(self):
        jax = pytest.importorskip("jax")
        import jax.numpy as jnp

        plant = Integrator()
        plant.params["k"] = 2.0
        controller = PropController()
        controller.params["Kp"] = 2.5

        dx = jax.jit(plant.f)(jnp.asarray([3.0]), jnp.asarray([4.0]))
        y = jax.jit(plant.h)(jnp.asarray([3.0]), jnp.asarray([4.0]))
        u_cmd = jax.jit(controller.ctl)(jnp.asarray([]), jnp.asarray([3.0, 1.0]))

        np.testing.assert_allclose(np.asarray(dx), [8.0])
        np.testing.assert_allclose(np.asarray(y), [3.0])
        np.testing.assert_allclose(np.asarray(u_cmd), [5.0])


if __name__ == "__main__":
    unittest.main()
