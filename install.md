# Install minilink (class quick start)

Minilink needs **Python 3.10+** (3.13 works well with conda). First pick a **toolkit tier**, then follow the conda or pip steps below.

After any install, jump to [Verify](#verify-it-works).

---

## Toolkit tiers — which one do I need?

| Tier | Install file(s) | You can run… |
| --- | --- | --- |
| **Basic** | [requirements.txt](requirements.txt) | Core sim/plot scripts, LQR, value iteration (`examples/demos/statespace/`, `examples/demos/planning/value_iteration/`) |
| **Full** | [requirements-full.txt](requirements-full.txt) or [environment.yml](environment.yml) | Everything in Basic **plus** intro/showcase notebooks, JAX trajopt/MPC, symbolic EOM, meshcat/plotly/pygame animators |
| **GRO860** | [requirements-gro860.txt](requirements-gro860.txt) or [environment-gro860.yml](environment-gro860.yml) | **UdeS GRO860 course** — Basic labs **plus** Jupyter and PPO RL notebooks (`examples/learn/teaching/pendulum_swing_up_vi_vs_lqr_vs_ppo.ipynb`, `drone_ppo_learn_to_fly.ipynb`, …) |

**Not sure?** GRO860 students → **GRO860 tier**. General minilink class with JAX notebooks → **Full**. Scripts only, no notebooks → **Basic**.

Upgrade path: Basic → `pip install -r requirements-gro860.txt` or `pip install -r requirements-full.txt` (both include Basic via `-r requirements.txt`).

---

## Conda install (recommended)

Conda is recommended because diagram rendering (`plot_diagram`) needs the Graphviz `dot` binary, bundled reliably via conda-forge.

### GRO860 toolkit

```bash
git clone https://github.com/alx87grd/minilink.git
cd minilink
conda env create -f environment-gro860.yml
conda activate minilink-gro860
conda env config vars set PYTHONPATH="$PWD"
conda deactivate && conda activate minilink-gro860
```

### Full toolkit

```bash
git clone https://github.com/alx87grd/minilink.git
cd minilink
conda env create -f environment.yml
conda activate minilink
conda env config vars set PYTHONPATH="$PWD"
conda deactivate && conda activate minilink
```

### Basic toolkit (minimal conda)

```bash
git clone https://github.com/alx87grd/minilink.git
cd minilink
conda create -n minilink-basic -c conda-forge python=3.13 numpy scipy matplotlib graphviz python-graphviz
conda activate minilink-basic
conda env config vars set PYTHONPATH="$PWD"
conda deactivate && conda activate minilink-basic
```

Add extras later:

```bash
# → GRO860
conda env create -f environment-gro860.yml   # or new env from the yml

# → Full
conda install -c conda-forge jax jaxlib jupyterlab ipykernel plotly sympy meshcat-python pygame gymnasium
pip install stable-baselines3 torch   # if you need PPO notebooks too
```

---

## Pip install

Works on any Python 3.10+ install (`venv`, `uv`, etc.).

**Graphviz:** pip installs the Python wrapper only — you also need the system `dot` binary:

- **Ubuntu / Debian:** `sudo apt install graphviz`
- **macOS (Homebrew):** `brew install graphviz`
- **Windows:** [Graphviz installer](https://graphviz.org/download/) — add `dot` to your PATH

Shared setup (replace `TIER` with `basic`, `full`, or `gro860`):

```bash
git clone https://github.com/alx87grd/minilink.git
cd minilink

python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

pip install --upgrade pip
pip install -r requirements-TIER.txt

export PYTHONPATH="$PWD"           # Windows (cmd): set PYTHONPATH=%CD%
                                   # Windows (PowerShell): $env:PYTHONPATH = (Get-Location)
```

Concrete commands:

```bash
# Basic
pip install -r requirements.txt

# Full (notebooks + JAX + teaching extras)
pip install -r requirements-full.txt

# GRO860 (notebooks + Gymnasium + Stable-Baselines3 + PyTorch)
pip install -r requirements-gro860.txt
```

**Editable install alternative** (same repo checkout):

```bash
pip install -e ".[jax,visualization,plotting,symbolic,rl]"   # Full-ish via pyproject extras
pip install jupyterlab ipykernel stable-baselines3 torch      # + GRO860 PPO stack
```

Pyproject extras: `jax`, `symbolic`, `visualization`, `plotting`, `ipopt`, `rl`, `dev`, `docs`.

---

## Google Colab (no local install)

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_minilink.ipynb)

JAX demos: [JAX showcase](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_jax.ipynb).

GRO860 PPO notebooks install `gymnasium` and `stable-baselines3` automatically on first run in Colab.

---

## Verify it works

From the repo root, with your env active and `PYTHONPATH` set:

```bash
python -c "from minilink import Pendulum; p = Pendulum(); print('OK:', p.n)"
```

Expected: `OK: 2`

**GRO860 tier** — check RL stack:

```bash
python -c "import gymnasium, stable_baselines3, torch; print('GRO860 RL OK')"
```

**Full tier** — check JAX:

```bash
python -c "import jax; print('JAX', jax.__version__)"
```

Optional simulation smoke test:

```bash
python -c "
from minilink import ImpedanceController, Pendulum
(ImpedanceController() @ Pendulum()).compute_trajectory(tf=1.0)
print('Simulation OK')
"
```

Open a notebook (Full or GRO860 tiers):

```bash
jupyter lab examples/learn/intro/showcase_minilink.ipynb          # Full
jupyter lab examples/learn/teaching/drone_ppo_learn_to_fly.ipynb  # GRO860
```

---

## Troubleshooting

**`ModuleNotFoundError: No module named 'minilink'`**

- Not in the repo root, or `PYTHONPATH` is unset. Fix: `cd minilink` then `export PYTHONPATH="$PWD"`.

**`ExecutableNotFound: failed to execute 'dot'` (Graphviz)**

- Install Graphviz binaries (conda tiers usually include them; pip users see Graphviz note above).

**`ModuleNotFoundError: stable_baselines3` / `torch`**

- You installed **Basic** or **Full** but opened a **GRO860** PPO notebook. Run: `pip install -r requirements-gro860.txt`.

**JAX / CUDA errors (Full tier)**

- Class demos use CPU JAX. Reinstall: `pip install -U "jax[cpu]"`.

**Ipopt / `cyipopt` (optional NLP solver, Full tier advanced labs)**

- Conda: `conda install -c conda-forge ipopt cyipopt`. Prefer conda over pip for this extra.

---

## Files in this repo

| File | Tier |
| --- | --- |
| [requirements.txt](requirements.txt) | Basic |
| [requirements-full.txt](requirements-full.txt) | Full (pip) |
| [requirements-gro860.txt](requirements-gro860.txt) | GRO860 (pip) |
| [environment.yml](environment.yml) | Full (conda) |
| [environment-gro860.yml](environment-gro860.yml) | GRO860 (conda) |
| [pyproject.toml](pyproject.toml) | Package metadata and optional `[extras]` |

Contributors: [README.md § Install](README.md#install).
