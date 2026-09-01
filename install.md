# Install minilink

Python **3.10+**. Two tiers — **Basic** or **Full** — each with conda, pip, and Colab.

A basic Python environment with NumPy, SciPy, and Matplotlib is enough for most
of the library. Skip `plot_diagram()` (needs Graphviz) and optional plotting or
optimization backends.

**Conda is recommended.** [`environment.yml`](environment.yml) installs all
optional extras so every minilink feature is available. It is the validated and
supported path.

**Pip warning.** Graphviz and Ipopt are non-Python binaries that pip cannot
install. Graphviz is only for `plot_diagram()`. Ipopt is an optional NLP
backend; trajopt defaults to SciPy. Install Graphviz first
([download](https://graphviz.org/download/), or `apt` / `brew`) if you need
diagrams, or use a `*-nographviz.txt` file below.

## Pick a tier

| | **Basic** | **Full** (recommended) |
| --- | --- | --- |
| **For** | Scripts: sim, trajectory plots, LQR, value iteration | Notebooks, JAX, animators, symbolic, PPO |
| **Conda** | [`environment-basic.yml`](environment-basic.yml) | [`environment.yml`](environment.yml) |
| **Pip** | [`requirements.txt`](requirements.txt) | [`requirements-full.txt`](requirements-full.txt) |
| **Pip, no Graphviz** | [`requirements-nographviz.txt`](requirements-nographviz.txt) | [`requirements-full-nographviz.txt`](requirements-full-nographviz.txt) |
| **Colab** | clone + path (NumPy/SciPy/Matplotlib already present) | clone + path + Full extras |

PPO notebooks need **Full**. VI and LQR run on **Basic**.

---

## Basic

### Conda

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda env create -f environment-basic.yml
conda activate minilink-basic
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink-basic
```

### Pip

pip installs the Graphviz Python wrapper; you also need the system `dot` binary
(`apt install graphviz`, `brew install graphviz`, or
[Windows installer](https://graphviz.org/download/)). Use
`requirements-nographviz.txt` to skip Graphviz and `plot_diagram()`.

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt                     # or requirements-nographviz.txt
export PYTHONPATH="$PWD"                            # Windows PS: $env:PYTHONPATH = (Get-Location)
```

### Colab

No local install. Open a notebook with **Open in Colab** (e.g.
[showcase](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_minilink.ipynb)).
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
`plot_diagram()` unless you install Graphviz on the runtime.

---

## Full

### Conda

Same as [README § Install](README.md#install):

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda env create -f environment.yml
conda activate minilink
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink
```

### Pip

Same Graphviz note as Basic (`dot` binary, or `requirements-full-nographviz.txt`).
Pip Full does not include Ipopt/`cyipopt` (conda Full does); trajopt still runs
with SciPy.

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements-full.txt                # or requirements-full-nographviz.txt
export PYTHONPATH="$PWD"                            # Windows PS: $env:PYTHONPATH = (Get-Location)
```

### Colab

Same clone + path as Basic, then Full extras Colab does not ship (JAX is already
on Colab). Example:
[JAX showcase](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_jax.ipynb).

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

## Troubleshooting

- **`No module named 'minilink'`** — set `PYTHONPATH` to the repo root (`export PYTHONPATH="$PWD"`).
- **`failed to execute 'dot'`** — install Graphviz, or reinstall with a `*-nographviz.txt` file and skip `plot_diagram()`.
- **Missing `stable_baselines3` / `torch`** — you need the **Full** tier for PPO notebooks.
- **`cyipopt` / Ipopt errors** — optional; Basic omits it. Use default SciPy solvers, or Full conda / `conda install -c conda-forge ipopt cyipopt`.
