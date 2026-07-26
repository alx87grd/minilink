"""Time integration, static forcing, and scheduled step simulation."""

from minilink.simulation.computer import Computer, StepSchedule

__all__ = [
    "Computer",
    "HybridSimResult",
    "HybridSimulator",
    "RealtimeSimulator",
    "StepSchedule",
]


def __getattr__(name: str):
    if name == "HybridSimulator":
        from minilink.simulation.hybrid_simulator import HybridSimulator

        return HybridSimulator
    if name == "HybridSimResult":
        from minilink.simulation.hybrid_simulator import HybridSimResult

        return HybridSimResult
    if name == "RealtimeSimulator":
        from minilink.simulation.realtime import RealtimeSimulator

        return RealtimeSimulator
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
