"""
Base class for deterministic search planners.

Search planners solve feasibility-style planning tasks. They may use
sampling internally, but the problem contract remains deterministic:
find a path or trajectory satisfying the declared sets.
"""

from __future__ import annotations

from minilink.planning.base import Planner
from minilink.planning.search.results import SearchResult


class SearchPlanner(Planner[SearchResult]):
    """
    Base class for deterministic search-style planners.

    Search planners use feasibility, sampling, and rollout/steering
    settings. They may omit a cost and still return a path or trajectory.
    """
