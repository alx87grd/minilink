# Revision Pass

Final pass after substantial changes. Goal: smaller, clearer diff aligned with
[agent.md](agent.md).

1. **Re-read diff** — scope creep, dead code, stale comments; preserve user edits.
2. **Simplify** — match local patterns; thin public APIs; lazy optional imports;
   fail clearly on unsupported paths.
3. **Math-first** — textbook locals in equation paths; conversion at boundaries only.
4. **Examples** — fold or update demos; runnable from repo root; no new guides unless asked.
5. **Docs** — sync [README.md](README.md) (user API), [DESIGN.md](DESIGN.md)
   (contracts), [ROADMAP.md](ROADMAP.md) (maturity only if changed). Keep
   [flows.md](flows.md) minimal—update chains, not step-by-step internals.
6. **Tests** — new behavior, regressions, optional-extra skip messages; benchmarks
   only when performance claims matter.
7. **Verify** — `ruff check .`, `ruff format --check .`, `pytest` (proportionate
   to risk); `MPLBACKEND=Agg` for headless graphics smoke.
8. **Handoff** — `git status`, no stray artifacts; short summary of changes and
   verification.
