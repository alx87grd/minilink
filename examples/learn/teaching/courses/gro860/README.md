# GRO860 — Commande optimale et apprentissage (UdeS)

Standalone Colab tutorials. The course notes link here (one-way); the notebooks
do not mention the course or each other.

Each notebook has an "Open in Colab" badge. On Colab the first cell clones the
repo and installs extra RL dependencies when needed.

## Labs

The three pendulum swing-up pages use the same plant and default quadratic cost,
and the same section order (plant → cost → planning problem → methods).

| Notebook | Topic |
| --- | --- |
| `labs/pendulum_swing_up_cost_function_vi.ipynb` | Cost function + value iteration |
| `labs/pendulum_swing_up_vi_vs_lqr.ipynb` | Value iteration vs LQR |
| `labs/pendulum_swing_up_vi_vs_lqr_vs_ppo.ipynb` | Value iteration vs LQR vs PPO |
| `labs/drone_ppo_learn_to_fly.ipynb` | Planar drone with PPO |

The two RL notebooks require the optional dependencies `gymnasium` and
`stable-baselines3` (training runs of ~10 minutes on the free Colab CPU tier).
