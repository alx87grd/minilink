# Install minilink

Python **3.10+**. Two tiers — **Basic** or **Full**.

A basic Python environment with NumPy, SciPy, and Matplotlib is enough for most
of the library. Skip `plot_diagram()` (needs Graphviz) and optional plotting or
optimization backends.

**Conda** from [`environment.yml`](environment.yml) is the Full local stack
(JAX, Ipopt, notebooks) that CI and the course validate.

Graphviz and Ipopt are non-Python binaries pip cannot install. Graphviz is only
for `plot_diagram()`. Ipopt is an optional NLP backend; trajopt defaults to SciPy.

## Pick a tier

| | **Basic** | **Full** (recommended locally) |
| --- | --- | --- |
| **For** | Scripts: sim, trajectory plots, LQR, value iteration | Notebooks, JAX, animators, symbolic, PPO |
| **PyPI** | `pip install minilink` | `pip install "minilink[full]"` |
| **Conda** | [`environment-basic.yml`](environment-basic.yml) | [`environment.yml`](environment.yml) |
| **From a clone** | `pip install -e .` | `pip install -e ".[full]"` |
| **Colab** | clone + path (NumPy/SciPy/Matplotlib already present) | clone + path + Full extras |

PPO notebooks need **Full**. VI and LQR run on **Basic**. Pip extras: `jax`,
`visualization`, `plotting`, `diagrams`, `symbolic`, `rl`, `ipopt`, `full`
(everything except `ipopt` — that extra needs a system Ipopt).

---

## Basic

### PyPI

```bash
pip install minilink
```

Until the `0.1.0` tag is on PyPI, install from a clone (`pip install -e .`
below) or from GitHub:

```bash
pip install "minilink @ git+https://github.com/alx87grd/minilink.git"
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
`plot_diagram()` unless you install Graphviz on the runtime. After `0.1.0` is
on PyPI, `%pip install minilink` is enough for Basic scripts.

---

## Full

### PyPI

```bash
pip install "minilink[full]"
```

Same GitHub fallback as Basic until the tag lands. Pip Full does not include
Ipopt/`cyipopt` (conda Full does); trajopt still runs with SciPy. For Ipopt:
`pip install "minilink[full,ipopt]"` after a system Ipopt.

### Conda

Same as [README § Install](README.md#install):

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda env create -f environment.yml
conda activate minilink
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink
```

### From a clone (pip)

Same Graphviz note as Basic (`dot` binary, or skip the `diagrams` extra).

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

## Publish from GitHub

The wheel is built by hatchling + hatch-vcs. Version comes from a **`0.*` git
tag** (`0.1.0`, not `v0.1.0`). The published package is the teaching surface
plus provisional bands; `minilink/experimental/` stays repo-only.

`.github/workflows/publish.yml` builds the sdist and wheel on a `0.*` tag (and
on `workflow_dispatch` for a dry build) and uploads to PyPI with Trusted
Publishing. One-time setup, on the maintainer account:

1. Create a GitHub Environment named `pypi` on `alx87grd/minilink`.
2. On PyPI, add a trusted publisher: owner `alx87grd`, repository `minilink`,
   workflow `publish.yml`, environment `pypi`. A pending publisher is enough
   before the first upload; the first successful publish creates the project.
3. Tag and push: `git tag 0.1.0 && git push origin 0.1.0`.

`workflow_dispatch` without a tag only builds; it does not upload.

---

## Troubleshooting

- **`No module named 'minilink'`** — `pip install minilink` (or `pip install -e .`
  at the repo root). On conda-only setups, set `PYTHONPATH` to the repo root
  (`export PYTHONPATH="$PWD"`).
- **`failed to execute 'dot'`** — install Graphviz, or skip `plot_diagram()` /
  the `diagrams` extra.
- **Missing `stable_baselines3` / `torch`** — you need the **Full** tier for PPO notebooks.
- **`cyipopt` / Ipopt errors** — optional; Basic omits it. Use default SciPy solvers, or Full conda / `conda install -c conda-forge ipopt cyipopt`.
- **PyPI 404 for `minilink`** — the first upload is the `0.1.0` tag. Until then
  use a clone or `pip install "minilink @ git+https://github.com/alx87grd/minilink.git"`.
