"""The System analysis shortcuts: which exist, and that each forwards exactly to its band function."""

import inspect
import unittest
from importlib import import_module
from unittest import mock

from minilink import InvertedPendulum, TransferFunction
from minilink.core.facades import DynamicSystemFacades, LTISystemFacades

# name -> (owner class, band function "module:attr", kind)
#   forward: the shortcut's parameters are the band function's after its first one
#   narrowed: a deliberate subset of them, each kept one identical to the band's
SHORTCUTS = {
    "linearize": (
        DynamicSystemFacades,
        "minilink.analysis.linearization:linearize",
        "forward",
    ),
    "find_equilibrium": (
        DynamicSystemFacades,
        "minilink.analysis.equilibria:find_equilibrium",
        "forward",
    ),
    "transfer_function": (
        DynamicSystemFacades,
        "minilink.analysis.frequency:transfer_function",
        "forward",
    ),
    "plot_bode": (
        DynamicSystemFacades,
        "minilink.analysis.frequency:plot_bode",
        "forward",
    ),
    "plot_root_locus": (
        DynamicSystemFacades,
        "minilink.analysis.frequency:plot_root_locus",
        "forward",
    ),
    "plot_pzmap": (
        DynamicSystemFacades,
        "minilink.analysis.frequency:plot_pzmap",
        "forward",
    ),
    "animate_modal": (
        DynamicSystemFacades,
        "minilink.analysis.modal:animate_modal",
        "forward",
    ),
    # traj defaults to the system's last simulation; the rest pass through **kwargs
    "plot_phase_plane": (
        DynamicSystemFacades,
        "minilink.graphical.phase_plane:plot_phase_plane",
        "narrowed",
    ),
    # a linear model needs no operating point: the linearization arguments are gone
    "plot_step_response": (
        LTISystemFacades,
        "minilink.analysis.time_response:plot_step_response",
        "narrowed",
    ),
}

# The data verbs stay band functions; the time response of a linearized model
# lives on LTISystem alone
REMOVED = (
    "bode",
    "pzmap",
    "margins",
    "root_locus",
    "step_response",
    "nyquist",
    "plot_nyquist",
    "region_of_attraction",
    "plot_region_of_attraction",
    "modal_analysis",
    "plot_step_response",
)


def band_function(target):
    module, attr = target.split(":")
    return getattr(import_module(module), attr)


def parameters(fn, *, drop_first):
    """``(name, kind, default)`` of each parameter, annotations ignored."""
    params = list(inspect.signature(fn).parameters.values())
    if drop_first:
        params = params[1:]
    return [(p.name, p.kind, p.default) for p in params]


class TestSystemShortcuts(unittest.TestCase):
    def test_the_kept_and_the_removed_shortcuts(self):
        plant = InvertedPendulum()
        for name, (owner, _, _) in SHORTCUTS.items():
            if owner is DynamicSystemFacades:
                self.assertTrue(callable(getattr(plant, name)), name)
        for name in REMOVED:
            self.assertFalse(hasattr(plant, name), name)
        self.assertTrue(callable(plant.linearize([0.0, 0.0]).plot_step_response))
        self.assertTrue(
            callable(TransferFunction([1.0], [1.0, 1.0]).plot_step_response)
        )

    def test_each_signature_matches_its_band_function(self):
        for name, (owner, target, kind) in SHORTCUTS.items():
            with self.subTest(name):
                shortcut = parameters(getattr(owner, name), drop_first=True)
                band = parameters(band_function(target), drop_first=True)
                if kind == "forward":
                    self.assertEqual(shortcut, band)
                else:
                    band_by_name = {p[0]: p for p in band}
                    for p in shortcut:
                        self.assertIn(p[0], band_by_name, f"{name}: {p[0]}")
                        if p[1] is inspect.Parameter.VAR_KEYWORD:
                            continue
                        self.assertEqual(p[2], band_by_name[p[0]][2], f"{name}: {p[0]}")

    def test_each_forward_passes_every_argument_under_its_own_name(self):
        plant = InvertedPendulum()
        for name, (owner, target, kind) in SHORTCUTS.items():
            if owner is not DynamicSystemFacades or kind != "forward":
                continue
            with self.subTest(name):
                module, attr = target.split(":")
                shortcut_params = inspect.signature(getattr(plant, name)).parameters
                sentinels = {p: object() for p in shortcut_params}
                positional = [
                    sentinels[p]
                    for p, spec in shortcut_params.items()
                    if spec.kind is inspect.Parameter.POSITIONAL_OR_KEYWORD
                ]
                keywords = {
                    p: sentinels[p]
                    for p, spec in shortcut_params.items()
                    if spec.kind is inspect.Parameter.KEYWORD_ONLY
                }
                band = band_function(target)
                with mock.patch(f"{module}.{attr}") as recorder:
                    getattr(plant, name)(*positional, **keywords)
                call = recorder.call_args
                bound = inspect.signature(band).bind(*call.args, **call.kwargs)
                first, *rest = inspect.signature(band).parameters
                self.assertIs(bound.arguments[first], plant)
                for p in rest:
                    self.assertIs(bound.arguments[p], sentinels[p], f"{name}: {p}")

    def test_each_docstring_names_its_band_function(self):
        for name, (owner, target, _) in SHORTCUTS.items():
            with self.subTest(name):
                module, attr = target.split(":")
                doc = inspect.getdoc(getattr(owner, name)) or ""
                self.assertIn(f"{module}.{attr}", doc)


if __name__ == "__main__":
    unittest.main()
