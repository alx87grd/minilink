# Install minilink (class quick start)

Minilink needs **Python 3.10+** (3.13 works well with conda). Pick **one** path below.

After any path, jump to [Verify](#verify-it-works).

---

## Which option should I use?

| You have… | Use |
| --- | --- |
| Anaconda or Miniconda | **Option 1** (recommended) |
| Plain Python + `pip` only | **Option 2** |
| Limited disk / slow network | **Option 3** (minimal) |
| No local install | **Option 4** (Colab) |

---

## Option 1 — Conda full environment (recommended)

One file installs core deps, JAX, notebooks, and optional teaching extras.

```bash
git clone https://github.com/alx87grd/minilink.git
cd minilink
conda env create -f environment.yml
conda activate minilink
conda env config vars set PYTHONPATH="$PWD"
conda deactivate && conda activate minilink
```

**Why conda?** Diagram rendering (`plot_diagram`) and some solvers need native libraries; conda-forge bundles them reliably on Windows, macOS, and Linux.

---

## Option 2 — Pip + virtual environment

Works on any Python 3.10+ install. Good if you already use `venv` or `uv`.

### 2a. Full classroom stack (notebooks + JAX + extras)

```bash
git clone https://github.com/alx87grd/minilink.git
cd minilink

python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

pip install --upgrade pip
pip install -r requirements-class.txt

export PYTHONPATH="$PWD"           # Windows (cmd): set PYTHONPATH=%CD%
                                   # Windows (PowerShell): $env:PYTHONPATH = (Get-Location)
```

### 2b. Core only (lighter)

```bash
git clone https://github.com/alx87grd/minilink.git
cd minilink

python -m venv .venv
source .venv/bin/activate

pip install --upgrade pip
pip install -r requirements.txt

export PYTHONPATH="$PWD"
```

Add extras anytime:

```bash
pip install jax jaxlib plotly sympy meshcat pygame jupyterlab
```

### 2c. Editable install from `pyproject.toml`

Same repo checkout; installs minilink as a package plus chosen extras:

```bash
pip install -e ".[jax,visualization,plotting,symbolic]"
pip install jupyterlab ipykernel
```

Available extras: `jax`, `symbolic`, `visualization`, `plotly`, `plotting`, `ipopt`, `rl`, `dev`, `docs`.

**Graphviz on pip:** install the system `dot` binary too, or diagrams that call Graphviz will fail:

- **Ubuntu / Debian:** `sudo apt install graphviz`
- **macOS (Homebrew):** `brew install graphviz`
- **Windows:** [Graphviz installer](https://graphviz.org/download/) — add `dot` to your PATH

---

## Option 3 — Conda minimal (smallest useful env)

Core simulation and plotting only; add extras later if a notebook needs them.

```bash
git clone https://github.com/alx87grd/minilink.git
cd minilink

conda create -n minilink -c conda-forge python=3.13 numpy scipy matplotlib graphviz python-graphviz
conda activate minilink
conda env config vars set PYTHONPATH="$PWD"
conda deactivate && conda activate minilink
```

Add teaching extras when needed:

```bash
conda install -c conda-forge jax jaxlib jupyterlab ipykernel plotly sympy meshcat-python pygame
```

---

## Option 4 — Google Colab (no local install)

Open the showcase notebook in Colab (runs in the browser):

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_minilink.ipynb)

Colab already has NumPy, SciPy, and Matplotlib. For JAX demos, use the [JAX showcase](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_jax.ipynb).

---

## Verify it works

From the repo root, with your env active and `PYTHONPATH` set to the repo (Options 1–3):

```bash
python -c "from minilink import Pendulum; p = Pendulum(); print('OK:', p.n)"
```

Expected output: `OK: 2`

Optional — run a short simulation:

```bash
python -c "
from minilink import ImpedanceController, Pendulum
(ImpedanceController() @ Pendulum()).compute_trajectory(tf=1.0)
print('Simulation OK')
"
```

Open the intro notebook (if Jupyter is installed):

```bash
jupyter lab examples/learn/intro/showcase_minilink.ipynb
```

---

## Troubleshooting

**`ModuleNotFoundError: No module named 'minilink'`**

- You are not in the repo root, or `PYTHONPATH` is not set to it.
- Fix: `cd minilink` then `export PYTHONPATH="$PWD"` (see your option above).

**`ExecutableNotFound: failed to execute 'dot'` (Graphviz)**

- Install Graphviz binaries (Option 1/3 conda usually fixes this; pip users see Option 2 Graphviz note).

**JAX / CUDA errors**

- Class demos use CPU JAX by default. If you installed a GPU build by mistake, reinstall CPU JAX: `pip install -U "jax[cpu]"`.

**Ipopt / `cyipopt` (optional NLP solver)**

- Not in the default class lists. Conda: `conda install -c conda-forge ipopt cyipopt`. Pip needs a working Ipopt library on your system — prefer conda for this extra.

---

## Files in this repo

| File | Purpose |
| --- | --- |
| [environment.yml](environment.yml) | Full conda dev + class environment |
| [requirements.txt](requirements.txt) | Pip core deps |
| [requirements-class.txt](requirements-class.txt) | Pip classroom stack |
| [pyproject.toml](pyproject.toml) | Package metadata and optional `[extras]` |

More detail for contributors: [README.md § Install](README.md#install).
