# Notebooks

Teaching and demo notebooks for minilink. Open any `.ipynb` from this folder on
GitHub, or browse the whole set in Colab:

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/tree/main/examples/notebooks)

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

The CI ``regression`` job executes every notebook's code cells (fail on error,
outputs discarded). Locally:

```bash
MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py
```

New notebooks under this folder must be added to
[`tests/demo_checks/notebook_manifest.json`](../../tests/demo_checks/notebook_manifest.json).

**Why only meshcat?** Colab already provides NumPy, SciPy, Matplotlib, and JAX-friendly stacks for these demos. Animation in the browser needs **meshcat**; the bootstrap installs that one extra dependency. Heavier local extras (`pip install -e ".[jax]"`, pygame, etc.) are for development on your machine, not required for the thin Colab path.

## Notebook list

| Notebook | Topic |
| --- | --- |
| [demo_showcase.ipynb](demo_showcase.ipynb) | Feature tour |
| [demo_compile_evaluator.ipynb](demo_compile_evaluator.ipynb) | Compile → evaluator |
| [demo_overview.ipynb](demo_overview.ipynb) | Extended lab |
| [demo_stateless_functional_jax.ipynb](demo_stateless_functional_jax.ipynb) | Stateless `f` and JAX tracing |
| [demo_optimization.ipynb](demo_optimization.ipynb) | Small NLPs |
| [demo_plots_animations_backends.ipynb](demo_plots_animations_backends.ipynb) | Plot / animate backends |
| [demo_mpc_circuit.ipynb](demo_mpc_circuit.ipynb) | Spatial MPC full stack (scene → cost → plan → hybrid deploy) |
| [driving_model_trajopt_analysis.ipynb](driving_model_trajopt_analysis.ipynb) | Car models + TrajOpt compare (obstacle + 90° corner) |
| [simulation_benchmark.ipynb](simulation_benchmark.ipynb) | ODE solver benchmarks |
