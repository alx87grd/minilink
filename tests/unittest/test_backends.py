"""Tests for :mod:`minilink.core.backends`."""

import importlib

import pytest

from minilink.core.backends import (
    BACKEND_AUTO,
    BACKEND_DIRECT,
    BACKEND_JAX,
    BACKEND_NUMPY,
    COMPILE_BACKENDS,
    SIMULATOR_BACKENDS,
    TRANSCRIPTION_BACKENDS,
    normalize_backend,
    require_jax_numpy,
)


def test_constants_have_expected_string_values():
    assert BACKEND_NUMPY == "numpy"
    assert BACKEND_JAX == "jax"
    assert BACKEND_AUTO == "auto"
    assert BACKEND_DIRECT == "direct"


def test_compile_simulator_transcription_backend_sets():
    assert set(COMPILE_BACKENDS) == {BACKEND_NUMPY, BACKEND_JAX}
    assert set(SIMULATOR_BACKENDS) == {BACKEND_NUMPY, BACKEND_JAX, BACKEND_AUTO}
    assert set(TRANSCRIPTION_BACKENDS) == {
        BACKEND_NUMPY,
        BACKEND_JAX,
        BACKEND_DIRECT,
    }


def test_normalize_backend_lowercases_and_validates():
    assert normalize_backend("NumPy") == BACKEND_NUMPY
    assert normalize_backend(" JAX ") == BACKEND_JAX
    assert normalize_backend(None) == BACKEND_NUMPY


def test_normalize_backend_rejects_unknown():
    with pytest.raises(ValueError):
        normalize_backend("torch")


def test_normalize_backend_auto_gated():
    with pytest.raises(ValueError):
        normalize_backend("auto")
    assert normalize_backend("auto", allow_auto=True) == BACKEND_AUTO


def test_normalize_backend_direct_gated():
    with pytest.raises(ValueError):
        normalize_backend("direct")
    assert normalize_backend("direct", allow_direct=True) == BACKEND_DIRECT


def test_require_jax_numpy_returns_module_when_jax_installed():
    pytest.importorskip("jax")
    jnp = require_jax_numpy()
    assert jnp.zeros(2).shape == (2,)


def test_simulator_re_exports_compile_backend_auto():
    sim = importlib.import_module("minilink.simulation.simulator")
    assert sim.COMPILE_BACKEND_AUTO == BACKEND_AUTO
