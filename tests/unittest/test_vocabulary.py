"""The vocabulary of RULES 2.7: no retired word on the public surface of the tools and the core."""

import dataclasses
import importlib
import inspect
import warnings

import pytest

# The facades the surface is read from; the plants' own physics symbols are their textbook's
FACADES = (
    "minilink",
    "minilink.core",
    "minilink.control",
    "minilink.estimation",
    "minilink.blocks",
    "minilink.analysis",
    "minilink.simulation",
    "minilink.optimization",
    "minilink.planning",
    "minilink.learning",
)
PLANT_PACKAGE = "minilink.dynamics"

# A retired word: the word of RULES 2.7 that replaced it, and the owners it is retired on
# (None: everywhere). A word keeps its other textbook meanings elsewhere: alpha is the CVaR
# level of Evaluation.cvar, N the reference gain of OutputReference. Each sweep adds its own.
RETIRED = {
    "solver": ("record", ("PlanningSolution",)),
    "algorithm": ("method", ("ReinforcementLearningPlanner", "TabularLearningPlanner")),
}


def public_objects():
    """Every exported class and function of the facades, once, plants aside."""
    found = {}
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        for name in FACADES:
            module = importlib.import_module(name)
            for attr in getattr(module, "__all__", []):
                obj = getattr(module, attr, None)
                home = getattr(obj, "__module__", "") or ""
                if not callable(obj) or home.startswith(PLANT_PACKAGE):
                    continue
                found[id(obj)] = (f"{name}.{attr}", obj)
    return list(found.values())


def words_of(qualname, obj):
    """The names a caller writes: parameters, public methods and fields, with where each sits."""
    words = []
    members = [(qualname, obj)]
    if inspect.isclass(obj):
        members = [(qualname, obj.__init__)]
        for member_name, member in vars(obj).items():
            if member_name.startswith("_"):
                continue
            words.append((member_name, f"{qualname}.{member_name}"))
            if callable(member):
                members.append((f"{qualname}.{member_name}", member))
        if dataclasses.is_dataclass(obj):
            words += [(f.name, f"{qualname}.{f.name}") for f in dataclasses.fields(obj)]
    for where, function in members:
        try:
            parameters = inspect.signature(function).parameters
        except (TypeError, ValueError):
            continue
        words += [(p, f"{where}({p}=)") for p in parameters]
    return words


def test_no_retired_word_on_the_public_surface():
    if not RETIRED:
        pytest.skip("no word retired yet")
    hits = sorted(
        f"{where}: {word!r} is retired, write {RETIRED[word][0]!r}"
        for qualname, obj in public_objects()
        for word, where in words_of(qualname, obj)
        if word in RETIRED and retired_on(qualname, RETIRED[word][1])
    )
    assert not hits, "\n".join(hits)


def retired_on(qualname, owners):
    """Whether a word is retired on this object: everywhere, or on the named owners."""
    if owners is None:
        return True
    return qualname.rsplit(".", 1)[-1] in owners
