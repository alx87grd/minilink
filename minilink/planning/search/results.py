"""
Result objects for search-style planners.

Search planners primarily return a feasible path or tree. A reconstructed
state-input trajectory may be attached when the search also produces timing
and control samples.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from minilink.core.trajectory import Trajectory


@dataclass(frozen=True)
class SearchResult:
    """
    Result returned by deterministic search planners.

    Parameters
    ----------
    path : object, optional
        Feasible path artifact, usually a sequence of states or nodes.
    traj : Trajectory, optional
        Reconstructed state-input trajectory when available.
    tree : object, optional
        Search tree or graph artifact.
    success : bool
        Whether the planner found a feasible path.
    message : str
        Human-readable planner status.
    stats : dict
        Planner statistics and debug metadata.
    """

    problem: Any | None = None
    path: Any | None = None
    traj: Trajectory | None = None
    tree: Any | None = None
    success: bool = False
    message: str = ""
    stats: dict[str, Any] = field(default_factory=dict)

    @property
    def has_path(self) -> bool:
        """Return ``True`` when a search path is available."""
        return self.path is not None

    @property
    def has_traj(self) -> bool:
        """Return ``True`` when a trajectory is available."""
        return self.traj is not None

    def require_path(self) -> Any:
        """Return the path or raise a clear error."""
        if self.path is None:
            raise ValueError("This search result does not contain a path")
        return self.path

    def require_traj(self) -> Trajectory:
        """Return the trajectory or raise a clear error."""
        if self.traj is None:
            raise ValueError("This search result does not contain a trajectory")
        return self.traj
