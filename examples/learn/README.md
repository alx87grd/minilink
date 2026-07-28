# Learn

Curriculum notebooks for minilink.

| Folder | Job |
| --- | --- |
| [`intro/`](intro/) | Learn **minilink** — numbered API chapters + showcase |
| [`teaching/topics/`](teaching/topics/) | Learn a **subject** (generic domain lessons) |
| [`teaching/courses/`](teaching/courses/) | Class / workshop packs (labs, problems) |

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/tree/main/examples/learn)

Each notebook also has its own **Open in Colab** badge under the title.

## Local vs Colab

| Where | How you get minilink | Extra install |
| --- | --- | --- |
| **Local** (recommended for development) | conda env `minilink` or `pip install -e ".[jax]"` from the repo root | none for most demos |
| **Google Colab** | bootstrap cell clones the repo and puts it on `sys.path` | **meshcat** only (`pip install` runs automatically) |

### Local

Use the repo’s `minilink` conda environment (see the root [README](../../README.md#install)),
or an editable install from the repository root. Run notebooks with that kernel;
no path hacks needed if the package is installed.

### Colab

1. Click **Open in Colab** on a notebook (or open this folder via the badge above).
2. **File → Save a copy in Drive** (GitHub-opened notebooks are read-only until you copy).
3. Run cells from the top. The first code cell detects Colab and:

   - sets `%matplotlib inline`
   - `git clone https://github.com/alx87grd/minilink`
   - adds `/content/minilink` to `sys.path`
   - `pip install -q meshcat`

On a normal local kernel, that cell is a no-op.

## CI smoke

The CI ``regression`` job executes notebooks under `intro/`, `teaching/topics/`,
and `examples/tooling/notebooks/` (not `courses/` or `projects/`). Locally:

```bash
MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py
```

New notebooks in those smoke roots are auto-discovered. Optional deps and longer
timeouts go in
[`tests/demo_checks/notebook_overrides.json`](../../tests/demo_checks/notebook_overrides.json).

**Why only meshcat?** Colab already provides NumPy, SciPy, Matplotlib, and JAX-friendly stacks for these demos. Animation in the browser needs **meshcat**; the bootstrap installs that one extra dependency. Heavier local extras (`pip install -e ".[jax]"`, pygame, etc.) are for development on your machine, not required for the thin Colab path.
