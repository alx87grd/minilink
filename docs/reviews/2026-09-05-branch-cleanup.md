# Branch cleanup — 2026-09-05 (S21)

Generated on `dev-fable`. Review, then run the commands you agree with;
nothing here is executed by the agent. Branches merged into `main` carry no
unique commits. Remote `cursor/*` branches are agent work branches.

## Local branches already merged into main

```bash
git branch -d backup/pre-phase5-cutover   # last: 2026-06-29
git branch -d claude/strange-nightingale   # last: 2025-04-03
git branch -d dev-antigravity   # last: 2026-03-19
git branch -d dev-compile   # last: 2026-04-05
git branch -d dev-doc   # last: 2026-05-30
git branch -d dev-eval   # last: 2026-07-12
git branch -d dev-eval2   # last: 2026-07-12
git branch -d dev-fable5   # last: 2026-06-13
git branch -d dev-folder   # last: 2026-04-26
git branch -d dev-graphical-refactor   # last: 2026-06-18
git branch -d dev-hybrid   # last: 2026-07-10
git branch -d dev-jax   # last: 2026-03-25
git branch -d dev-mechanics   # last: 2026-04-07
git branch -d dev-mjx   # last: 2026-07-25
git branch -d dev-model   # last: 2026-07-25
git branch -d dev-mpc   # last: 2026-07-04
git branch -d dev-mpc-v2   # last: 2026-07-16
git branch -d dev-opti   # last: 2026-05-04
git branch -d dev-planning   # last: 2026-04-30
git branch -d dev-plotting-backend   # last: 2026-05-11
git branch -d dev-rl   # last: 2026-08-28
git branch -d dev-simulator   # last: 2026-04-16
git branch -d dev-sources   # last: 2026-03-26
git branch -d dev-sys-reorganizing   # last: 2026-04-24
git branch -d dev-test   # last: 2026-07-17
git branch -d dev-trajopt   # last: 2026-04-30
git branch -d dev-v2   # last: 2025-04-02
git branch -d feature/console-graphviz-matplotlib   # last: 2026-07-28
git branch -d merge-alex-sim   # last: 2026-07-17
git branch -d refactor-v4   # last: 2026-06-29
git branch -d test-meshcat   # last: 2026-03-25
```

## Local branches NOT merged (keep unless you know they are dead)

```
2026-02-26  dev  (1 commits ahead of main)
2026-06-26  kinematic-refactor-v2  (1 commits ahead of main)
2026-06-26  refactor-frames  (1 commits ahead of main)
2026-06-29  refactor-v3  (4 commits ahead of main)
2026-09-01  dev-alex  (3 commits ahead of main)
2026-09-05  dev-fable  (16 commits ahead of main)
```

## Remote cursor/* branches

Merged into origin/main → safe to delete; the rest are listed with dates for your call.

```bash
git push origin --delete cursor/articulated-mechanism-plan-404b   # merged, last: 2026-07-28
git push origin --delete cursor/consolidate-dev-alex-merge-92ed   # merged, last: 2026-07-26
git push origin --delete cursor/control-block-plan-review-ffd7   # merged, last: 2026-08-29
git push origin --delete cursor/cursor-test-update-dev-alex-8149   # merged, last: 2026-04-25
git push origin --delete cursor/dev-model-to-dev-alex-8d26   # merged, last: 2026-07-23
git push origin --delete cursor/docs-interface-readme-3717   # merged, last: 2026-05-29
git push origin --delete cursor/euler-solver-split-53db   # merged, last: 2026-07-12
git push origin --delete cursor/fix-ci-jax-graphviz-fa58   # merged, last: 2026-07-10
git push origin --delete cursor/hybrid-discrete-simulation-plan-5121   # merged, last: 2026-07-02
git push origin --delete cursor/hybrid-discrete-simulation-plan-78c2   # merged, last: 2026-07-05
git push origin --delete cursor/integration-regression-53db   # merged, last: 2026-07-12
git push origin --delete cursor/jax-bicycle-servo-torque-inputs-3c63   # merged, last: 2026-07-17
git push origin --delete cursor/mpc-bicycle-straight-line-dd94   # merged, last: 2026-06-18
git push origin --delete cursor/mpc-controller-architecture-f16d   # merged, last: 2026-07-15
git push origin --delete cursor/mpc-post-refactor-cleanup-plan-552f   # merged, last: 2026-07-15
git push origin --delete cursor/neural-blocks-plan-d009   # merged, last: 2026-06-19
git push origin --delete cursor/numpy-mpc-rebuild-5608   # merged, last: 2026-07-16
git push origin --delete cursor/organize-demo-scripts-672e   # merged, last: 2026-04-30
git push origin --delete cursor/patch-hybrid-plan-phase6-28e0   # merged, last: 2026-07-08
git push origin --delete cursor/phase-0-closure-0b92   # merged, last: 2026-07-08
git push origin --delete cursor/phase3-collision-docs-1557   # merged, last: 2026-06-30
git push origin --delete cursor/planning-pipeline-architecture-doc-a701   # merged, last: 2026-07-01
git push origin --delete cursor/post-refactor-cleanup-1557   # merged, last: 2026-06-29
git push origin --delete cursor/pyro-port-gap-todo-d009   # merged, last: 2026-06-20
git push origin --delete cursor/roadmap-architecture-review-c1ea   # merged, last: 2026-07-24
git push origin --delete cursor/servo-inputs-uy-parity-e11d   # merged, last: 2026-07-18
git push origin --delete cursor/test-benchmark-cleanup-1557   # merged, last: 2026-06-30
git push origin --delete cursor/test-benchmark-consolidation-plan-f14c   # merged, last: 2026-07-16
```

```
2026-04-25  cursor/cursor-test-update-8149  (not merged)
2026-05-10  cursor/diagram-editor-mvp-a189  (not merged)
2026-06-09  cursor/cleanup-simsimsim-pr-cba4  (not merged)
2026-07-01  cursor/mpc-spatial-scene-guide-c445  (not merged)
2026-07-01  cursor/workspace-cost-field-plots-c445  (not merged)
2026-07-08  cursor/unify-compile-api-28e0  (not merged)
2026-07-09  cursor/hybrid-review-pass-443e  (not merged)
2026-07-15  cursor/planning-ui-simplification-552f  (not merged)
2026-07-27  cursor/ci-notebook-fix-6f77  (not merged)
2026-07-27  cursor/lorenz-attractor-demo-6f77  (not merged)
2026-07-27  cursor/three-body-astro-demo-6f77  (not merged)
2026-07-28  cursor/symbolic-fd-speed-test-b631  (not merged)
2026-08-31  cursor/install-guide-followups-1567  (not merged)
2026-08-31  cursor/student-install-guide-1567  (not merged)
```
