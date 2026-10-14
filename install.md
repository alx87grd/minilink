# Install minilink

Python **3.10+**. Two tiers — **Basic** or **Full**.

A basic Python environment with NumPy, SciPy, and Matplotlib is enough for most
of the library. Skip `plot_diagram()` (needs Graphviz) and optional plotting or
optimization backends.

**Conda** from [`environment.yml`](environment.yml) is the recommended install:
the whole Full stack (JAX, Ipopt, Jupyter, Stable-Baselines3) the courses use.

Graphviz and Ipopt are non-Python binaries pip cannot install. Graphviz is only
for `plot_diagram()`. Ipopt is an optional NLP backend; trajopt defaults to SciPy.

## Pick a tier

| | **Basic** | **Full** (recommended locally) |
| --- | --- | --- |
| **For** | Scripts: sim, trajectory plots, LQR, value iteration | Notebooks, JAX, animators, symbolic, PPO |
| **Conda** | [`environment-basic.yml`](environment-basic.yml) | [`environment.yml`](environment.yml): everything, notebooks included |
| **PyPI** | `pip install minilink` | `pip install "minilink[full]"`: the libraries only |
| **From a clone** | `pip install -e .` | `pip install -e ".[full]"`: the libraries only |
| **Colab** | clone + path (NumPy/SciPy/Matplotlib already present) | clone + path + Full extras |

VI and LQR run on **Basic**. Pip's `full` extra installs every library extra
(JAX, plotting, 3-D animation, symbolic math, diagrams, Gymnasium) but no
Jupyter, PyTorch or Stable-Baselines3: run the notebooks, the
Stable-Baselines3 ones included, on the conda environment. Pip extras: `jax`,
`visualization`, `plotting`, `diagrams`, `symbolic`, `rl`, `ipopt`, `full`
(every one except `ipopt`, which needs a system Ipopt).

---

## Basic

### PyPI

```bash
pip install minilink
```

### Conda

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda env create -f environment-basic.yml
conda activate minilink-basic
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink-basic
```

### From a clone (pip)

pip installs the Graphviz Python wrapper when you add the `diagrams` extra; you
also need the system `dot` binary (`apt install graphviz`, `brew install graphviz`,
or [Windows installer](https://graphviz.org/download/)). Skip that extra to skip
`plot_diagram()`.

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e .
```

### Colab

No local install. Open a notebook with **Open in Colab** (e.g.
[showcase](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/showcase_minilink.ipynb)).
Paste as the **first code cell** if you build your own notebook:

```python
# Local conda: minilink already installed. Colab: clone + path.
import sys

if "google.colab" in sys.modules:
    get_ipython().run_line_magic("matplotlib", "inline")
    get_ipython().system("git clone https://github.com/alx87grd/minilink")
    sys.path.insert(0, "/content/minilink")
```

Colab already ships NumPy, SciPy, and Matplotlib (the Basic stack). Skip
`plot_diagram()` unless you install Graphviz on the runtime. In a notebook of
your own, `%pip install minilink` is enough for Basic scripts.

---

## Full

### PyPI

```bash
pip install "minilink[full]"
```

Pip Full installs the libraries only: no Jupyter, PyTorch, Stable-Baselines3 or
Ipopt (the conda environment has all four). Trajectory optimization still runs
with SciPy. For Ipopt: `pip install "minilink[full,ipopt]"` after a system Ipopt.

### Conda

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda env create -f environment.yml
conda activate minilink
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink
```

### From a clone (pip)

Same Graphviz note as Basic (`dot` binary, or skip the `diagrams` extra), and
the same libraries as pip Full above.

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e ".[full]"
```

### Colab

Same clone + path as Basic, then Full extras Colab does not ship (JAX is already
on Colab). Example:
[JAX showcase](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/showcase_jax.ipynb).

```python
# Local conda: minilink already installed. Colab: clone + path + Full extras.
import sys

if "google.colab" in sys.modules:
    get_ipython().run_line_magic("matplotlib", "inline")
    get_ipython().system("git clone https://github.com/alx87grd/minilink")
    sys.path.insert(0, "/content/minilink")
    get_ipython().system(
        "pip install -q meshcat pygame plotly sympy gymnasium stable-baselines3"
    )
```

---

## Developer clone (contributing)

Any tier above, then the dev tools and the two git hooks, **once per clone**:

```bash
pip install -e ".[dev]"      # conda env: already installed
pre-commit install
nbstripout --install
git config filter.nbstripout.extrakeys "metadata.kernelspec metadata.language_info metadata.colab metadata.accelerator cell.metadata.colab cell.metadata.id cell.metadata.outputId cell.metadata.executionInfo"
```

`pre-commit install` runs ruff and the `nbstripout` hook on every commit.
`nbstripout --install` sets the git filter that strips notebook outputs on commit
and diff; the `extrakeys` line makes it strip the same volatile metadata as the
hook (kernelspec, language_info, Colab keys), the keys a local run or Colab
rewrites. Without this step, running a notebook makes `git status` dirty, and
after a commit made elsewhere with the filter, `git pull` refuses on notebooks
you never edited. Hooks and filter are per clone: redo them on every machine.

---

## Troubleshooting

- **`No module named 'minilink'`** — `pip install minilink` (or `pip install -e .`
  at the repo root). On conda-only setups, set `PYTHONPATH` to the repo root
  (`export PYTHONPATH="$PWD"`).
- **`failed to execute 'dot'`** — install Graphviz, or skip `plot_diagram()` /
  the `diagrams` extra.
- **Missing `stable_baselines3` / `torch`** — the Stable-Baselines3 notebooks
  run on the conda environment ([`environment.yml`](environment.yml)); pip's
  `full` extra does not install them.
- **`cyipopt` / Ipopt errors** — optional; Basic omits it. Use default SciPy solvers, or Full conda / `conda install -c conda-forge ipopt cyipopt`.
- **Notebooks show as modified after running them, or `git pull` refuses on
  notebooks you never edited** — the per-clone notebook filter is missing or
  stripping fewer keys than the hook: run the [Developer clone](#developer-clone-contributing)
  block. The files are byte-identical to HEAD; `git -c filter.nbstripout.clean=cat pull --ff-only`
  gets the pull through once, then the filter and the stored notebooks agree.
