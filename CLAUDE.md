# Minilink — agent entry point

> **Before any work, check the remote.** Run `git fetch` and `git status -sb`. If the branch
> is behind `origin/<branch>`, stop and tell the maintainer before reading, planning or
> editing anything else. Work done on a stale local branch has been lost several times
> (AGENTS: Before you start).

Read [AGENTS.md](AGENTS.md) now: it is the workflow contract (what to read,
what needs the maintainer, the local CI gate). It applies to every agent,
whichever tool loads this file.

Then, as AGENTS.md directs: [CONSTITUTION.md](CONSTITUTION.md) before any
interaction (questions and reviews included), [RULES.md](RULES.md) before editing
Python.

Until you have read those, four things hold:

- Never revert or "polish" the maintainer's manual edits in `examples/`,
  notebooks, or scratch files.
- Never remove a feature or a user-importable name without an explicit
  maintainer decision.
- Math reads like the textbook page. Every step of an algorithm is visible in the body,
  linear algebra included, and the plumbing goes to helpers. No `self.` in math lines.
  `dp.py` is the reference (AGENTS: the textbook rule).
- Run `ruff check .` and `ruff format --check .` before any push.
