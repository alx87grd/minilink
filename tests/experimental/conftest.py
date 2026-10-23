"""The experimental tier's tests: off the CI merge gate, run by the nightly job (ROADMAP §2).

They share the optional-extra markers and the JAX skip of ``tests/unittest``.
"""

from tests.unittest.conftest import pytest_collection_modifyitems  # noqa: F401
