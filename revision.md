# Revision Pass

Use this as the final pass after a substantial implementation, refactor, or
architecture change. The goal is not to make the diff bigger. The goal is to
make the final diff smaller, clearer, easier to review, and aligned with
`minilink`'s math-first design rules.

## 1. Re-read The Actual Diff

- Run `git status --short` and inspect the touched files.
- Review the implementation as a reviewer, not as the author.
- Identify accidental scope creep, dead code, duplicate helpers, debug prints,
  one-off examples, temporary files, and stale comments.
- Preserve user-authored changes. Do not revert unrelated dirty work.

## 2. Simplify Architecture And Interfaces

- Prefer the existing package layout and local patterns over new abstractions.
- Remove interface ceremony that does not enforce a real contract.
- Merge helpers when they only wrap one line or obscure the data flow.
- Split helpers only when it makes the math or boundary conversion clearer.
- Keep public APIs thin. Add knobs only when there is a clear caller need.
- Check that optional features stay optional and imports remain lazy.
- If a backend or mode is not meant to support a workflow, fail clearly rather
  than letting it enter a bad partial path.

## 3. Make The Code Math-First

- Equation paths should read like textbook math: map inputs once, then use
  explicit locals such as `x`, `u`, `y`, `A`, `B`, `q`, `v`, `M`, `K`.
- Keep `np.asarray`, shape normalization, Python `float(...)`, plotting
  conversion, and I/O at boundaries.
- Prefer explicit loops and named temporaries over clever Python when the code
  represents systems, signals, trajectories, or optimization equations.
- Delete comments that narrate obvious Python. Keep comments only for
  non-obvious conventions, side channels, or architecture tradeoffs.

## 4. Clean Files And Examples

- Delete new files that do not earn their place.
- Fold tiny examples into an existing demo when that is clearer than adding a
  new script.
- Update old examples that would otherwise point users at stale APIs.
- Keep demo scripts flat and runnable from the repo root.
- Do not add broad markdown guides unless the user asked for one.

## 5. Sync Contracts And Docs

- Update `README.md` when install commands, user-facing capabilities, or common
  examples change.
- Update `DESIGN.md` when package layout, public contracts, backend semantics,
  or architecture expectations change.
- Update `ROADMAP.md` only when maturity, priorities, or planned work actually
  changed.
- Make docs precise about limitations. Do not imply experimental code is a
  mature default path.

## 6. Strengthen Tests Where Risk Changed

- Add tests for new public behavior, boundary errors, and optional dependency
  messages.
- Add regression tests for bugs fixed during the implementation.
- Avoid broad snapshot-style tests unless they protect a real contract.
- If a change affects optional extras, tests should skip gracefully when the
  extra is missing and still validate the missing-extra error message where
  possible.
- For performance-related changes, add or update a benchmark only if the claim
  matters to the user-facing decision.

## 7. Verify

Run verification in proportion to the risk:

```bash
ruff check .
ruff format --check .
pytest
```

If the environment lacks dev tools or optional extras, report that clearly.
Useful fallbacks:

```bash
python -m py_compile <touched-python-files>
pytest <focused-test-files>
```

For user-facing graphical changes, also run or document the relevant smoke demo.
Use headless-safe settings such as `MPLBACKEND=Agg` when appropriate.

## 8. Final Review Before Handoff

- Re-run `git status --short`.
- Skim `git diff --stat` and the final diff for accidental churn.
- Confirm no generated caches, notebooks, screenshots, or temporary artifacts
  were added by mistake.
- Confirm the final answer names what changed, what was verified, and what could
  not be run.
- Keep the final response short and concrete.
