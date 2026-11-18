"""Capture or check the plotting baseline: python docs/plans/plotting-run/baseline.py [--check]."""

import glob
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
TESTS = [
    "tests/unittest/test_graphics.py",
    "tests/unittest/test_planning_solution.py",
    "tests/unittest/test_control_plots.py",
    "tests/unittest/test_dynamic_programming.py",
    "tests/unittest/test_control_analysis.py",
    "tests/unittest/test_planning.py",
    "tests/unittest/test_diagrams.py",
    "tests/unittest/test_analysis_lyapunov.py",
    "tests/unittest/test_core.py",
    "tests/unittest/test_simulation.py",
    "tests/unittest/test_planner_params.py",
    "tests/unittest/test_step_discrete.py",
    "tests/unittest/test_costs_optimizer.py",
    "tests/unittest/test_sampled_loop.py",
    "tests/unittest/test_mppi.py",
    "tests/unittest/test_mpc.py",
    "tests/unittest/test_estimation.py",
    "tests/unittest/test_system_shortcuts.py",
    "docs/plans/plotting-run/test_baseline_extras.py",
]


def capture(out):
    """Run the plotting tests with the recording plugin; merge the workers' records into out."""
    for old in glob.glob(out + ".*.json"):
        os.remove(old)
    env = dict(
        os.environ,
        BASELINE_OUT=out,
        MPLBACKEND="Agg",
        MINILINK_HEADLESS="1",
        SDL_VIDEODRIVER="dummy",
        PYTHONPATH=HERE + os.pathsep + ROOT,
    )
    cmd = [
        sys.executable,
        "-m",
        "pytest",
        "-q",
        "-n",
        "8",
        "--dist",
        "loadfile",
        "-p",
        "baseline_plugin",
        "-p",
        "no:cacheprovider",
        *TESTS,
    ]
    result = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True, text=True)
    lines = [
        l
        for l in result.stdout.splitlines()
        if " passed" in l or " failed" in l or "error" in l.lower()[:20]
    ]
    print(lines[-1] if lines else result.stdout[-1500:] + result.stderr[-1500:])
    merged = {}
    for part in glob.glob(out + ".*.json"):
        with open(part) as fh:
            merged.update(
                {k: v for k, v in json.load(fh).items() if "Realtime" not in k}
            )
        os.remove(part)
    with open(out, "w") as fh:
        json.dump(merged, fh, sort_keys=True, indent=1)
    return merged


def compare(reference, current):
    """The tests whose data record moved, appeared or disappeared."""
    moved = [
        k
        for k in reference
        if k in current and reference[k]["data"] != current[k]["data"]
    ]
    gone = [k for k in reference if k not in current]
    new = [k for k in current if k not in reference]
    return moved, gone, new


if __name__ == "__main__":
    reference_path = os.path.join(HERE, "baseline.json")
    if "--check" in sys.argv:
        current = capture(os.path.join(HERE, "current.json"))
        with open(reference_path) as fh:
            reference = json.load(fh)
        moved, gone, new = compare(reference, current)
        print(
            f"figures: {len(current)} tests; moved {len(moved)}, gone {len(gone)}, new {len(new)}"
        )
        for k in moved:
            print("MOVED", k)
        for k in gone:
            print("GONE ", k)
        sys.exit(1 if moved else 0)
    capture(reference_path)
