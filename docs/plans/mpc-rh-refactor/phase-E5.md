# Phase E5 — Retire PoC leaves

**Status:** stub (expand before coding)  
**Depends on:** demos/tests on RH ([E3](phase-E3.md)+)

## Goal

Remove or thin-wrap `MPCStatelessController` / `MPCStatefulController` after
everything uses `RecedingHorizonController` (pre-1.0 rename cleanly). Keep
warm_start / overlays tools.

## Exit

One controller product: `RecedingHorizonController`.

## Gate

Full MPC / hybrid unittest set green.

## Outline (expand when starting)

| Step | Work |
| --- | --- |
| E5.1 | Point remaining call sites at RH export |
| E5.2 | Delete or deprecate PoC leaf modules |
| E5.3 | Shrink hybrid tests to RH surface |

Next: [phase-E6.md](phase-E6.md).
