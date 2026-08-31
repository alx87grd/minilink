# Install minilink

Python **3.10+**. Conda recommended (bundles Graphviz for `plot_diagram()`).

**Graphviz is not core.** Simulation, trajectory plots, phase planes, and animation work without it. Only **`plot_diagram()`** (block-diagram topology) needs Graphviz — skip it if you use the no-Graphviz pip files below.

**Ipopt / `cyipopt` is not in Basic pip** (or GRO860). Trajopt and NLP default to **SciPy** (`scipy_slsqp`) — you will not fail without it. Only install Ipopt if you explicitly set `method="ipopt"` (Full conda [`environment.yml`](environment.yml) includes it).

## Pick a tier

| Tier | For | Conda | Pip | Pip (no Graphviz) |
| --- | --- | --- | --- | --- |
| **Basic** | Scripts (sim, LQR, value iteration) | see below | `requirements.txt` | `requirements-nographviz.txt` |
| **Full** | Notebooks + JAX + animators | `environment.yml` | `requirements-full.txt` | `requirements-full-nographviz.txt` |
| **GRO860** | UdeS course (VI, LQR, PPO) | `environment-gro860.yml` | `requirements-gro860.txt` | `requirements-gro860-nographviz.txt` |

GRO860 students → **GRO860**. JAX/showcase notebooks → **Full**. Otherwise → **Basic**.

---

## Conda (recommended)

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink

# Pick one:
conda env create -f environment-gro860.yml   # GRO860
conda env create -f environment.yml          # Full

conda activate minilink-gro860               # or: minilink
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink-gro860
```

**Basic** (no yml file):

```bash
conda create -n minilink-basic -c conda-forge python=3.13 numpy scipy matplotlib graphviz python-graphviz
conda activate minilink-basic
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink-basic
```

Omit `graphviz python-graphviz` from the line above if you will not call `plot_diagram()`.

---

## Pip

**With Graphviz** — pip installs the Python wrapper; you also need the system `dot` binary (`apt install graphviz`, `brew install graphviz`, or [Windows installer](https://graphviz.org/download/)).

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements-gro860.txt              # or requirements-full.txt / requirements.txt
export PYTHONPATH="$PWD"                            # Windows PS: $env:PYTHONPATH = (Get-Location)
```

**Without Graphviz** — same tiers, no `graphviz` package or `dot` binary needed; avoid `plot_diagram()`:

```bash
pip install -r requirements-gro860-nographviz.txt   # or requirements-full-nographviz.txt / requirements-nographviz.txt
```

**Colab:** [showcase notebook](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_minilink.ipynb) — no local install.

---

## Verify

```bash
python -c "from minilink import Pendulum; print('OK:', Pendulum().n)"
```

Expected: `OK: 2`

---

## Troubleshooting

- **`No module named 'minilink'`** — set `PYTHONPATH` to the repo root (`export PYTHONPATH="$PWD"`).
- **`failed to execute 'dot'`** — install Graphviz, or reinstall with a `*-nographviz.txt` file and skip `plot_diagram()`.
- **Missing `stable_baselines3` / `torch`** — you need the **GRO860** tier for PPO notebooks.
- **`cyipopt` / Ipopt errors** — optional; Basic/GRO860 pip omit it. Use default SciPy solvers, or `conda install -c conda-forge ipopt cyipopt` / Full conda env if you need Ipopt.

Contributors: editable install and dev extras in [README.md § Install](README.md#install).
