# Install minilink

Python **3.10+**. Conda recommended (bundles Graphviz for `plot_diagram()`).

## Pick a tier

| Tier | For | Conda | Pip |
| --- | --- | --- | --- |
| **Basic** | Scripts only (sim, LQR, value iteration) | see below | `requirements.txt` |
| **Full** | Notebooks + JAX + animators | `environment.yml` | `requirements-full.txt` |
| **GRO860** | UdeS course (VI, LQR, PPO) | `environment-gro860.yml` | `requirements-gro860.txt` |

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

---

## Pip

Pip needs the system Graphviz `dot` binary (`apt install graphviz`, `brew install graphviz`, or [Windows installer](https://graphviz.org/download/)).

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements-gro860.txt              # or requirements-full.txt / requirements.txt
export PYTHONPATH="$PWD"                            # Windows PS: $env:PYTHONPATH = (Get-Location)
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
- **`failed to execute 'dot'`** — install Graphviz binaries (conda envs include them).
- **Missing `stable_baselines3` / `torch`** — you need the **GRO860** tier for PPO notebooks.

Contributors: editable install and dev extras in [README.md § Install](README.md#install).
