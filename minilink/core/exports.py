"""Lazy band facades: one export table per package, names resolved on first use."""

from __future__ import annotations

from importlib import import_module
from typing import Any, Callable

###############################################################################
# Public API
###############################################################################


def lazy_facade(
    module_globals: dict[str, Any],
    exports: dict[str, tuple[str, str]],
    *,
    modules: dict[str, str] | None = None,
) -> tuple[list[str], Callable[[str], Any], Callable[[], list[str]]]:
    """Wire a package ``__init__`` as a lazy facade over ``exports``.

    ``exports`` maps a public name to ``(module path, attribute)``; ``modules``
    maps a name to a subpackage exposed the same way but kept out of
    ``__all__``. Returns ``(__all__, __getattr__, __dir__)`` for the caller
    to bind::

        __all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
    """
    module_name = module_globals["__name__"]
    modules = dict(modules or {})

    # The module hook: a subpackage, or one export of its module, imported on first use
    def __getattr__(name: str) -> Any:
        if name in modules:
            value = import_module(modules[name])
        else:
            try:
                module_path, attr = exports[name]
            except KeyError:
                raise AttributeError(
                    f"module {module_name!r} has no attribute {name!r}"
                ) from None
            value = getattr(import_module(module_path), attr)

        # Bound in the package, so the next lookup never reaches the hook
        module_globals[name] = value

        return value

    # The names a listing shows: the bound ones, the exports and the subpackages
    def __dir__() -> list[str]:
        return sorted(set(module_globals) | set(exports) | set(modules))

    return sorted(exports), __getattr__, __dir__
