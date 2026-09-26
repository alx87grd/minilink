"""Online estimators: blocks that infer hidden states or parameters from ``(u, y)`` (planned, step P4).

Placement rule: online, in-the-loop inference lives here; *offline* fitting
over logged data is a verb and lives in ``identification/``. Design factories
take plain arrays in and return this package's blocks out. The design is
step P4 of docs/plans/TODO.md (ROADMAP.md §5); its follow-ups are a row there.
"""
