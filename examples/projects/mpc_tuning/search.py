"""Phased MPC hyperparameter search for one-lap time.

Run from repo root::

    PYTHONPATH=. python examples/projects/mpc_tuning/search.py --model dynamic_engine --phase 0
    PYTHONPATH=. python examples/projects/mpc_tuning/search.py --model kinematic --phase 1 --max-trials 40
"""

from __future__ import annotations

import argparse
import copy
import json
import random
import sys
from pathlib import Path
from typing import Any

from examples.projects.mpc_tuning.harness import (
    CONFIGS_DIR,
    RESULTS_DIR,
    append_jsonl,
    is_valid_lap,
    load_json,
    run_lap,
    save_json,
)
from examples.projects.mpc_tuning.models import MODEL_KEYS, default_config

# Phase 1 search spaces
HORIZONS = [1.0, 1.5, 2.0, 2.5, 3.0]
STEPS = [8, 10, 12, 15, 20]
MAXITERS = [50, 100, 150]
FTOLS = [0.1, 0.05, 0.01]
U_TARGETS = [8.0, 10.0, 12.0, 15.0, 20.0]
WEIGHTS = [10.0, 20.0, 40.0]


def _seed_path(model: str) -> Path:
    return CONFIGS_DIR / f"seed_{model}.json"


def _best_path(model: str) -> Path:
    return CONFIGS_DIR / f"best_{model}.json"


def _ensure_seeds() -> None:
    CONFIGS_DIR.mkdir(parents=True, exist_ok=True)
    for key in MODEL_KEYS:
        path = _seed_path(key)
        if not path.exists():
            save_json(path, default_config(key))


def _load_base(model: str) -> dict[str, Any]:
    path = _seed_path(model)
    if path.exists():
        return load_json(path)
    return default_config(model)


def _log_trial(
    model: str,
    phase: int,
    trial: int,
    config: dict[str, Any],
    lap,
) -> None:
    record = {
        "phase": phase,
        "trial": trial,
        "model": model,
        "config": config,
        "result": lap.to_dict(),
    }
    append_jsonl(model, record)
    status = "VALID" if lap.valid else "fail"
    lt = f"{lap.lap_time:.2f}s" if lap.lap_time is not None else "—"
    med = f"{lap.median_solve_s:.3f}s" if lap.median_solve_s is not None else "—"
    err = f" err={lap.error}" if lap.error else ""
    print(
        f"[{model} p{phase} #{trial}] {status} lap={lt} "
        f"mean_e={lap.mean_path_error:.2f} med_solve={med} "
        f"nlp={lap.nlp_success_rate:.0%} progress={lap.progress_s:.1f}{err}",
        flush=True,
    )


def _score_lap(lap, *, require_realtime: bool) -> float | None:
    """Lower is better; None if invalid."""
    if not is_valid_lap(lap, require_realtime=require_realtime):
        return None
    assert lap.lap_time is not None
    return float(lap.lap_time)


# --- Phase 0: Engine baseline -------------------------------------------------


def phase0_engine(*, max_trials: int = 20) -> dict[str, Any]:
    model = "dynamic_engine"
    base = _load_base(model)
    candidates: list[dict[str, Any]] = []

    # Conservative scripted grid around Phase-0 knobs.
    for u in (8.0, 10.0, 12.0):
        for path_w, cor_w in ((20.0, 25.0), (40.0, 40.0), (40.0, 60.0)):
            for p_frac in (0.35, 0.5):
                for r_p in (2e-7, 5e-8):
                    for method in ("scipy_slsqp",):
                        cfg = copy.deepcopy(base)
                        cfg.update(
                            {
                                "U_TARGET": u,
                                "PATH_COST_WEIGHT": path_w,
                                "CORRIDOR_COST_WEIGHT": cor_w,
                                "P_ubar_frac": p_frac,
                                "R_P": r_p,
                                "optimizer_method": method,
                                "MPC_HORIZON": 2.0,
                                "MPC_STEPS": 10,
                                "maxiter": 100,
                                "ftol": 0.1,
                                "Q_vx": 5.0,
                            }
                        )
                        candidates.append(cfg)

    # If SLSQP struggles, try ipopt once per promising setting.
    ipopt_bases = candidates[:6]
    for cfg0 in ipopt_bases:
        cfg = copy.deepcopy(cfg0)
        cfg["optimizer_method"] = "ipopt"
        candidates.append(cfg)

    random.seed(0)
    random.shuffle(candidates)
    candidates = candidates[:max_trials]

    best_cfg = None
    best_lap = None
    for i, cfg in enumerate(candidates):
        lap = run_lap(model, cfg)
        _log_trial(model, 0, i, cfg, lap)
        if lap.valid and (
            best_lap is None
            or (lap.lap_time is not None and lap.lap_time < best_lap.lap_time)
        ):
            best_cfg, best_lap = cfg, lap
            if lap.nlp_success_rate >= 0.8:
                print(f"Phase 0 success: lap={lap.lap_time:.2f}s", flush=True)
                save_json(_seed_path(model), best_cfg)
                return best_cfg

    if best_cfg is not None:
        save_json(_seed_path(model), best_cfg)
        print(
            f"Phase 0 best valid lap={best_lap.lap_time:.2f}s "
            f"(nlp={best_lap.nlp_success_rate:.0%})",
            flush=True,
        )
        return best_cfg

    print("Phase 0: no valid lap found", flush=True)
    return base


# --- Phase 1: coarse grid -----------------------------------------------------


def _sample_phase1(model: str, base: dict[str, Any], rng: random.Random) -> dict[str, Any]:
    cfg = copy.deepcopy(base)
    cfg["MPC_DT"] = 0.2
    cfg["MPC_HORIZON"] = rng.choice(HORIZONS)
    cfg["MPC_STEPS"] = rng.choice(STEPS)
    cfg["maxiter"] = rng.choice(MAXITERS)
    cfg["ftol"] = rng.choice(FTOLS)
    cfg["U_TARGET"] = rng.choice(U_TARGETS)
    cfg["PATH_COST_WEIGHT"] = rng.choice(WEIGHTS)
    cfg["CORRIDOR_COST_WEIGHT"] = rng.choice(WEIGHTS)
    if model == "dynamic_engine":
        # Prefer method used in seed; allow ipopt with small probability if seed used it.
        seed_method = str(base.get("optimizer_method", "scipy_slsqp"))
        if seed_method == "ipopt" and rng.random() < 0.4:
            cfg["optimizer_method"] = "ipopt"
        else:
            cfg["optimizer_method"] = "scipy_slsqp"
        cfg["P_ubar_frac"] = rng.choice([0.25, 0.35, 0.5, 0.6])
        cfg["R_P"] = rng.choice([5e-8, 1e-7, 2e-7, 5e-7])
        cfg["Q_vx"] = rng.choice([2.0, 5.0, 10.0])
    return cfg


def phase1_grid(model: str, *, max_trials: int = 40) -> dict[str, Any]:
    base = _load_base(model)
    rng = random.Random(hash(model) & 0xFFFFFFFF)
    best_cfg = copy.deepcopy(base)
    best_score = None
    for i in range(max_trials):
        cfg = _sample_phase1(model, base, rng)
        lap = run_lap(model, cfg)
        _log_trial(model, 1, i, cfg, lap)
        score = _score_lap(lap, require_realtime=False)
        if score is not None and (best_score is None or score < best_score):
            best_score = score
            best_cfg = cfg
            print(f"  new best lap={score:.2f}s", flush=True)
    save_json(_seed_path(model), best_cfg)
    # Also stash interim best for later phases.
    interim = CONFIGS_DIR / f"phase1_{model}.json"
    save_json(interim, best_cfg)
    return best_cfg


# --- Phase 2: realtime refine -------------------------------------------------


def phase2_realtime(model: str, *, max_trials: int = 20) -> dict[str, Any]:
    path = CONFIGS_DIR / f"phase1_{model}.json"
    base = load_json(path) if path.exists() else _load_base(model)
    # Collect valid phase-1 configs from JSONL with median_solve <= 0.25
    candidates = [copy.deepcopy(base)]
    jsonl = RESULTS_DIR / f"{model}.jsonl"
    if jsonl.exists():
        with jsonl.open(encoding="utf-8") as f:
            for line in f:
                rec = json.loads(line)
                if int(rec.get("phase", -1)) != 1:
                    continue
                res = rec.get("result", {})
                if not res.get("valid"):
                    continue
                med = res.get("median_solve_s")
                if med is not None and float(med) <= 0.25:
                    candidates.append(copy.deepcopy(rec["config"]))

    # Unique-ish by (horizon, steps, maxiter, ftol)
    seen = set()
    uniq = []
    for c in candidates:
        key = (
            c.get("MPC_HORIZON"),
            c.get("MPC_STEPS"),
            c.get("maxiter"),
            c.get("ftol"),
            c.get("U_TARGET"),
        )
        if key in seen:
            continue
        seen.add(key)
        uniq.append(c)
    candidates = uniq[:12]

    best_cfg = None
    best_score = None
    trial = 0
    for cfg0 in candidates:
        if trial >= max_trials:
            break
        # Refine toward realtime: fewer steps / shorter horizon / coarser ftol.
        refinements = [cfg0]
        for steps in (cfg0.get("MPC_STEPS", 10), 8, 10, 12):
            for hor in (cfg0.get("MPC_HORIZON", 2.0), 1.5, 2.0):
                for ftol in (cfg0.get("ftol", 0.1), 0.1, 0.05):
                    cfg = copy.deepcopy(cfg0)
                    cfg["MPC_DT"] = 0.2
                    cfg["MPC_STEPS"] = int(steps)
                    cfg["MPC_HORIZON"] = float(hor)
                    cfg["ftol"] = float(ftol)
                    refinements.append(cfg)
        # Dedup refinements
        seen_r = set()
        ref_uniq = []
        for c in refinements:
            k = (c["MPC_HORIZON"], c["MPC_STEPS"], c["ftol"], c.get("maxiter"))
            if k in seen_r:
                continue
            seen_r.add(k)
            ref_uniq.append(c)

        for cfg in ref_uniq:
            if trial >= max_trials:
                break
            lap = run_lap(model, cfg)
            _log_trial(model, 2, trial, cfg, lap)
            trial += 1
            score = _score_lap(lap, require_realtime=True)
            if score is not None and (best_score is None or score < best_score):
                best_score = score
                best_cfg = cfg
                print(f"  RT best lap={score:.2f}s med={lap.median_solve_s:.3f}s", flush=True)

    if best_cfg is None:
        # Fall back: take best valid even if slightly over 0.2s and tighten.
        print(f"Phase 2: no RT-valid config for {model}; keeping phase1 seed", flush=True)
        best_cfg = base
    save_json(CONFIGS_DIR / f"phase2_{model}.json", best_cfg)
    save_json(_seed_path(model), best_cfg)
    return best_cfg


# --- Phase 3: lap-time local search -------------------------------------------


def _perturb(cfg: dict[str, Any], model: str, rng: random.Random) -> dict[str, Any]:
    out = copy.deepcopy(cfg)
    knobs = [
        ("U_TARGET", [max(6.0, out["U_TARGET"] + d) for d in (-2, -1, 0, 1, 2)]),
        (
            "PATH_COST_WEIGHT",
            [max(5.0, out["PATH_COST_WEIGHT"] * f) for f in (0.5, 0.75, 1.0, 1.25, 1.5)],
        ),
        (
            "CORRIDOR_COST_WEIGHT",
            [
                max(5.0, out["CORRIDOR_COST_WEIGHT"] * f)
                for f in (0.5, 0.75, 1.0, 1.25, 1.5)
            ],
        ),
        (
            "MPC_HORIZON",
            [max(1.0, min(3.0, out["MPC_HORIZON"] + d)) for d in (-0.5, 0.0, 0.5)],
        ),
    ]
    key, choices = rng.choice(knobs)
    out[key] = rng.choice(choices)
    if model == "dynamic_engine":
        if rng.random() < 0.3:
            out["P_ubar_frac"] = rng.choice([0.25, 0.35, 0.5, 0.6])
        if rng.random() < 0.3:
            out["R_P"] = rng.choice([5e-8, 1e-7, 2e-7, 5e-7])
        if rng.random() < 0.3:
            out["Q_vx"] = rng.choice([2.0, 5.0, 8.0, 10.0])
    out["MPC_DT"] = 0.2
    return out


def phase3_laptime(model: str, *, max_trials: int = 40) -> dict[str, Any]:
    path = CONFIGS_DIR / f"phase2_{model}.json"
    base = load_json(path) if path.exists() else _load_base(model)
    rng = random.Random(42 + (hash(model) & 0xFFFF))
    current = copy.deepcopy(base)
    lap0 = run_lap(model, current)
    _log_trial(model, 3, 0, current, lap0)
    best_cfg = current
    best_score = _score_lap(lap0, require_realtime=True)
    if best_score is None:
        # Accept valid without RT as starting point for exploration.
        best_score = _score_lap(lap0, require_realtime=False)

    for i in range(1, max_trials):
        cfg = _perturb(current, model, rng)
        lap = run_lap(model, cfg)
        _log_trial(model, 3, i, cfg, lap)
        score = _score_lap(lap, require_realtime=True)
        if score is None:
            continue
        if best_score is None or score < best_score:
            best_score = score
            best_cfg = cfg
            current = cfg
            print(f"  new best lap={score:.2f}s", flush=True)
        elif score <= best_score * 1.05:
            # Mild accept for local exploration
            current = cfg

    save_json(CONFIGS_DIR / f"phase3_{model}.json", best_cfg)
    return best_cfg


# --- Phase 4: freeze ----------------------------------------------------------


def phase4_freeze(models: list[str] | None = None) -> None:
    models = list(models or MODEL_KEYS)
    rows = []
    for model in models:
        # Prefer phase3 → phase2 → seed
        for name in (f"phase3_{model}.json", f"phase2_{model}.json", f"seed_{model}.json"):
            path = CONFIGS_DIR / name
            if path.exists():
                cfg = load_json(path)
                break
        else:
            cfg = default_config(model)
        lap = run_lap(model, cfg)
        _log_trial(model, 4, 0, cfg, lap)
        save_json(_best_path(model), cfg)
        rows.append((model, cfg, lap))

    print("\n=== Phase 4 summary ===")
    print(
        f"{'model':<16} {'lap_s':>8} {'mean_e':>8} {'med_solve':>10} "
        f"{'nlp':>6} {'valid':>6} {'RT':>4}"
    )
    for model, cfg, lap in rows:
        lt = f"{lap.lap_time:.2f}" if lap.lap_time is not None else "—"
        med = f"{lap.median_solve_s:.3f}" if lap.median_solve_s is not None else "—"
        print(
            f"{model:<16} {lt:>8} {lap.mean_path_error:>8.2f} {med:>10} "
            f"{lap.nlp_success_rate:>5.0%} {str(lap.valid):>6} {str(lap.realtime_ok):>4}"
        )
        print(
            f"  horizon={cfg.get('MPC_HORIZON')} steps={cfg.get('MPC_STEPS')} "
            f"U={cfg.get('U_TARGET')} path_w={cfg.get('PATH_COST_WEIGHT')} "
            f"method={cfg.get('optimizer_method')}"
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="MPC lap tuning search")
    parser.add_argument(
        "--model",
        choices=[*MODEL_KEYS, "all"],
        default="all",
        help="Plant key (or all)",
    )
    parser.add_argument(
        "--phase",
        type=int,
        choices=[0, 1, 2, 3, 4],
        required=True,
        help="Search phase",
    )
    parser.add_argument("--max-trials", type=int, default=None)
    args = parser.parse_args(argv)

    _ensure_seeds()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    models = list(MODEL_KEYS) if args.model == "all" else [args.model]

    if args.phase == 0:
        if "dynamic_engine" not in models and args.model != "all":
            print("Phase 0 is Engine-only; use --model dynamic_engine", file=sys.stderr)
            return 2
        n = args.max_trials if args.max_trials is not None else 20
        phase0_engine(max_trials=n)
        return 0

    if args.phase == 4:
        phase4_freeze(models)
        return 0

    defaults = {1: 40, 2: 20, 3: 40}
    n = args.max_trials if args.max_trials is not None else defaults[args.phase]
    for model in models:
        if args.phase == 0:
            continue
        print(f"\n======== {model} phase {args.phase} ({n} trials) ========", flush=True)
        if args.phase == 1:
            phase1_grid(model, max_trials=n)
        elif args.phase == 2:
            phase2_realtime(model, max_trials=n)
        elif args.phase == 3:
            phase3_laptime(model, max_trials=n)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
