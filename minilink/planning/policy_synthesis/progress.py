"""Terminal progress reports for the value-iteration table builds and sweeps."""

import time

import numpy as np

PROGRESS_PCT_STEP = 2.0
_PROGRESS_WIDTH = 72

# Public API


def progress(items, prefix, verbose, *, unit="nodes", per_item=1):
    """
    Iterate over ``items`` while reporting a table build; silent unless ``verbose``.

    Prints a header, an in-place percent line about every 2% with elapsed
    time and ETA, and a completion line. Each item counts as ``per_item``
    units, so a loop over the nodes of an ``(N, A)`` table reports its ``N A``
    pairs.
    """
    total = len(items) * per_item
    start = time.time()
    state = {"enabled": verbose}
    done = 0

    if verbose:
        print(f"{prefix}.. {total:,} {unit}", flush=True)

    for item in items:
        yield item

        if verbose:
            for _ in range(per_item):
                done += 1
                maybe_print_build_progress(
                    done, total, start, prefix=prefix, unit=unit, state=state
                )

    if verbose:
        print_build_complete(prefix, time.time() - start, f"({total:,} {unit})")


class SweepLog:
    """
    Bookkeeping of the backward sweeps: the pyro-style report and the ``(t, J, pi)`` history.

    Verbose solves print a banner, one line per sweep (a progress line for a
    fixed horizon) and a closing line. ``history`` is the recorded list, or
    ``None`` when the options do not ask for it.
    """

    def __init__(self, options, max_sweeps, stop_on_tol):
        self.verbose = options.verbose
        self.max_sweeps = max_sweeps
        self.stop_on_tol = stop_on_tol
        self.history = [] if options.record_history else None
        self.clock = time.time()
        self.state = {"enabled": options.verbose}

        if not self.verbose:
            return

        if stop_on_tol:
            print(f"\nComputing backward DP iterations until dJ<{options.tol:.2f}:")
            print("---------------------------------------------------------")
        else:
            print(f"\nComputing {max_sweeps} backward DP iterations:")
            print("-----------------------------------------")

    def start(self, t, J, pi):
        """Record the cost-to-go at the final time."""
        self.record(t, J, pi)

    def sweep(self, k, t, J, J_next, pi):
        """Report and record one completed sweep."""
        self.record(t, J, pi)

        if not self.verbose:
            return

        if self.stop_on_tol:
            J, J_next = np.asarray(J), np.asarray(J_next)
            delta = J - J_next
            elapsed = time.time() - self.clock
            print(
                f"{k:4d} t:{t:7.2f} Elapsed:{elapsed:7.2f} "
                f"max:{J.max():7.2f} dmax:{delta.max():7.2f} dmin:{delta.min():7.2f}"
            )
        else:
            maybe_print_build_progress(
                k,
                self.max_sweeps,
                self.clock,
                prefix="Computing backward DP",
                unit="sweeps",
                state=self.state,
            )

    def done(self, k, delta):
        """Close the report."""
        if not self.verbose:
            return

        if self.stop_on_tol:
            print()
            print("Bellman equation solved!")
        else:
            elapsed = time.time() - self.clock
            print_build_complete(
                "Computing backward DP", elapsed, f"({k} sweeps, delta={delta:.3f})"
            )

    def record(self, t, J, pi):
        """Keep a copy of ``(t, J, pi)`` when history is asked for."""
        if self.history is not None:
            self.history.append((t, np.array(J), np.array(pi)))


def print_build_progress(done, total, start, *, prefix, unit="pairs"):
    """Overwrite one terminal line with percent, elapsed time, and ETA."""
    elapsed = time.time() - start
    pct = 100.0 * done / total if total else 100.0
    eta = elapsed / done * (total - done) if 0 < done < total else 0.0
    msg = f"{prefix}: {pct:5.1f}%  {elapsed:5.0f}s  ETA ~{eta:.0f}s"
    pad = " " * max(0, _PROGRESS_WIDTH - len(msg))
    print(f"\r{msg}{pad}", end="", flush=True)


def maybe_print_build_progress(done, total, start, *, prefix, unit, state):
    """Print at most once per ``PROGRESS_PCT_STEP`` percent (plus final 100%)."""
    if not state.get("enabled", True) or total <= 0:
        return state
    pct = 100.0 * done / total
    if done < total:
        next_at = state.get("next_pct", PROGRESS_PCT_STEP)
        if pct + 1e-9 < next_at:
            return state
        while next_at <= pct:
            next_at += PROGRESS_PCT_STEP
        state["next_pct"] = next_at
    print_build_progress(done, total, start, prefix=prefix, unit=unit)
    return state


def print_build_complete(prefix, elapsed, detail):
    """Finish a throttled progress line without leaving wrapped tail text."""
    msg = f"{prefix}.. completed in {elapsed:4.2f} sec  {detail}"
    pad = " " * max(0, _PROGRESS_WIDTH - len(msg))
    print(f"\r{msg}{pad}")
