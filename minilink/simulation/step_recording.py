"""Record sampled port signals along a discrete state rollout."""

from __future__ import annotations

import numpy as np

from minilink.core.step_rollout import StepRollout


def record_boundary_outputs(
    rollout: StepRollout, evaluator, *, params=None
) -> StepRollout:
    """
    Sample compiled boundary outputs at each step index in ``rollout``.

    Evaluator rollouts return state and input samples only. Call this helper from
    simulation orchestrators when ``StepRollout.signals`` should hold boundary
    port histories — for example after a synchronous reference rollout outside
    :class:`~minilink.simulation.computer.Computer`.

    Parameters
    ----------
    rollout
        State-only rollout from :meth:`~minilink.core.compile.evaluators.step_rollout.StepRolloutMixin.rollout`.
    evaluator
        Compiled step evaluator exposing ``outputs`` / ``outputs_p`` and ``p``.
    params
        When set, use ``outputs_p`` with this nested params pytree.

    Returns
    -------
    StepRollout
        Copy of ``rollout`` with ``signals`` filled from boundary outputs.
    """
    if not getattr(evaluator, "p", 0):
        return rollout

    n_samples = rollout.n_samples
    signals: dict[str, np.ndarray] = {}

    for i in range(n_samples):
        x_i = rollout.x[:, i]
        u_i = rollout.u[:, i]
        k_i = float(rollout.k[i])
        if params is None:
            out = evaluator.outputs(x_i, u_i, k_i)
        else:
            out = evaluator.outputs_p(x_i, u_i, k_i, params)

        for port_id, values in out.items():
            arr = np.asarray(values, dtype=float).reshape(-1)
            if port_id not in signals:
                signals[port_id] = np.zeros((arr.size, n_samples), dtype=float)
            signals[port_id][:, i] = arr

    return StepRollout(k=rollout.k, x=rollout.x, u=rollout.u, signals=signals)
