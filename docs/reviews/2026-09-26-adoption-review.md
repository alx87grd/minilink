# Adoption review — 2026-09-26

Question: what minilink still lacks before a colleague can build a course on it. Read-only
audit of README, install.md, the docs site, examples/, CI and the recent reviews. It drove the
v0.9 / v1.0 rungs of ROADMAP §5.4 and §5.5. Since written, TB-a 3b closed the four analysis
bugs it listed.

## Verdict in one paragraph
The core is good work. There is one `System` behind every tool, the code reads like the
textbook, there are about 50 catalog plants and 41 Colab-ready notebooks, the frequency /
LQR / DP / trajopt / MPC / RL stack is strong, and the NumPy-only Basic tier installs with
`pip install minilink`. What keeps a colleague from adopting it is **not** the engine. There
are three reasons:
1. **Nothing promises stability.** Public names still change weekly, removals are clean cuts,
   there is no `DeprecationWarning` anywhere in the code, there is no CHANGELOG, and the Colab
   cells clone `main`.
2. **Two content holes that every course hits:** state estimation and discrete-time design.
3. **Nothing is packaged for an instructor.** The docs site is autodoc only, the teaching
   material is filed under UdeS course codes, and internal governance vocabulary shows in
   user-facing docs.

The current ROADMAP defines v1.0 by internal foundation questions (S29, S31, S37, S32, V1).
Those questions matter, but they are *prerequisites* to a freeze, not what makes a colleague
say yes.

## Recommended v1.0 definition
> **v1.0 = a colleague can build a course on it and trust it for three years.**
> The foundations are settled, the API is frozen with a real deprecation policy, the standard
> control syllabus is covered, material is adoptable outside UdeS, and it runs on students'
> laptops.

## The six v1.0 themes (ranked by how much each one blocks adoption)

### 1. Trust contract: stability you can put in a syllabus (largest blocker)
- **Semantic versioning with teeth.** Removing or renaming a teaching-surface name costs one
  minor release with a `DeprecationWarning` shim. This contradicts RULES ("no deprecated
  aliases"), so it is the maintainer's call. Proposal: before 1.0 the rule stays as it is;
  from 1.0 on, shims become mandatory.
- **A frozen, versioned root export list.** There are about 138 names today. The registry
  already exists; add a snapshot test that fails on any unannounced change.
- **A CHANGELOG.md** that users read. At present `pyproject` points at ROADMAP.
- **Pinned installs in material.** Colab cells run `pip install minilink==X.Y`, not
  `git clone main`. Fix install.md, which still says 0.1.0 is not on PyPI, and give one clear
  install story: pip first, conda for the full stack.
- **Settle the foundation questions that can break the API *before* the freeze:** S31
  (sampled loop as a `System`), S29 (derived `x0`), S37 (evaluator/solver layering) and S30
  (glyph/solid rename). Anything that would break names after 1.0 has to be decided now.
  Everything else in §5.4 (V1 differentiable cost, S36 iLQR, S32 unification if it is
  additive) can go after 1.0.

### 2. Standard-syllabus coverage: the holes every colleague hits
| Gap | Why it blocks | Existing roadmap hook |
| --- | --- | --- |
| **Estimation**: Luenberger, steady-state Kalman, LQG composition (EKF as a stretch) | Every state-space course; the package is an empty placeholder | P4, B3 (after the disturbance convention) |
| **Discrete time**: exact ZOH/Tustin `c2d`, z-plane pzmap, discrete step/Bode | Every digital-control course; P6 holds it | P6. Recommend un-holding it for 1.0 even though GRO501 does not examine it |
| **S/T/PS/CS, N matrix, LQI** | Loop shaping and tracking; cheap to add | P5, P8 |
| **Classic first plants**: DC motor, tank/thermal (first order plus delay), ball-and-beam, differential drive, 3-D quadrotor | The usual first lecture of classical control and mobile robotics | S64 area and C-wave catalog |
| **Trajectory generation**: polynomial, trapezoidal, minimum jerk | Feeds every computed-torque lab | C4 |
| **Identification**: least squares, ARX, step fit | Lab courses | C4 (`fitting.py`) |
| **General serial chain** (DH or URDF → RNEA/ABA) | Robotics beyond the UR5 | new; could stay post-1.0 |

### 3. Correctness of the numbers students check against the textbook
- Close the open analysis bugs from the 2026-09-26 audit (TODO step 3b): the `root_locus`
  singular gain, `step_info` on a response that never settles, MIMO silently truncated to
  channel 1, and inconsistent rank tolerances.
- Add a **textbook-validation suite**: a set of Dorf/Ogata/Franklin worked examples asserted
  numerically (margins, step specs, LQR gains, Kalman gains). GRO501 gate 3 is the seed; make
  it course-neutral and permanent. It is also the best advertising artifact.
- Harden SciPy/Ipopt trajopt to TRL 6. It is the one teaching tool still marked "harden".
- Continue S66 (wiring mistakes fail at wiring time) and S65 (one owner per rule). Add a
  `MinilinkError` hierarchy (wiring, shape, solver) so messages and autograders can catch a
  category.

### 4. Runs on students' laptops
- Add **Windows and macOS legs to CI**, at minimum the Basic tier plus a notebook smoke run.
  Today everything runs on ubuntu-latest, and students use Windows and macOS.
- Add coverage measurement (report only, no gate) so gaps are visible between audit sweeps.
- Make sure the Basic tier (NumPy, SciPy, Matplotlib) has graceful paths for graphviz,
  pygame and meshcat. The graphviz decision is already made.

### 5. Adoptable teaching material (instructor-facing packaging)
- **A rendered docs site**: tutorials rendered with myst-nb or nbsphinx, a plant catalog
  gallery with pictures, and a short concept guide ("What is a System", composition, tools
  as verbs). Today the site says the user guide is the notebooks on GitHub.
- **Course-neutral topic modules.** `teaching/topics/` has 7 pages, one each for classical
  control and robotics. Grow it into roughly 10–15 self-contained labs, each with objectives,
  prerequisites, a time estimate and a starter/solution split. Keep the UdeS course folders
  as "case studies".
- **An "For instructors" README section and page**: how to pin a version, run in Colab,
  adapt a lab, and report a bug.
- The classical and robotics halves of the material are thin compared with optimal control
  and RL, even where the library already has the functions.

### 6. Public face and community
- Keep internal vocabulary out of user-facing files. That means TRL, lanes, wave or step
  ids, "Agent MVP", links to CONSTITUTION/RULES/AGENTS from the README, and "smoked /
  nightly sweep" in examples/README. Consider moving AGENTS/CLAUDE/RULES under `docs/dev/`
  or `.github/`.
- Trim the README to a teaching quickstart first. JAX benchmarks, compile tiers and the Gym
  bridge go lower down or to the site. Fix the example that uses `np` without importing it.
- Add CONTRIBUTING, CITATION.cff with a Zenodo DOI (V2), issue templates and a Code of
  Conduct. JOSS is a strong credibility signal once 1.0 is frozen.
- Mark the provisional bands (hybrid/step, MPC, realtime, spatial) visibly in the docs.
  Decide whether `StepSystem`, `ZOHHold` and `control.mpc` belong on the frozen root
  surface or behind an explicit `minilink.experimental`-style marker.

## Proposed re-slicing of the rungs (to discuss)
- **v0.2 (Oct):** keep GRO501. Add theme 3's analysis bugs and the Windows/macOS CI legs,
  which are cheap and protect GRO501 students.
- **v0.3 (Dec):** GMC714, plus the textbook objects (A1–A3), plus trajectory generation and
  the classic plants.
- **v0.4 / 0.9 (early 2027): "freeze candidate".** Decide S31/S29/S37/S30, adopt the
  deprecation policy, add the CHANGELOG, pin installs, build the docs site, and add discrete
  time and identification.
- **v1.0 (after two cohorts):** freeze the API, publish the textbook-validation suite and
  the instructor labs, add CITATION/DOI, and decide on JOSS. V1 and S36 become 1.x features.

## Decisions this surfaces for the maintainer
1. Is v1.0 defined as "adoptable by a colleague" (recommended) or as "foundations closed"?
2. The deprecation policy: shims from 1.0 on, against RULES "no deprecated aliases".
3. Is the discrete-time / z-domain tier in scope for 1.0 (recommended yes)?
4. Are the step/hybrid/MPC names on the frozen root surface or explicitly provisional?
5. Should the governance files (AGENTS, CLAUDE, RULES, CONSTITUTION) stay at the repo root?

