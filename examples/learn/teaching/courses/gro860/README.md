# GRO860 — Commande optimale et apprentissage (UdeS)

Colab-ready notebooks backing the GRO860 course exercises. Each notebook has an
"Open in Colab" badge; on Colab the first cell clones the repo and installs the
extra RL dependencies when needed.

## Labs

| Notebook | Course exercise | Library features |
| --- | --- | --- |
| `labs/cost_function_pendulum.ipynb` | C.1.5 Fonction de coût pour un pendule | value iteration, custom `CostFunction`, lookup controller |
| `labs/lqr_vs_vi_pendulum.ipynb` | Intro/ch.2 demo — LQR vs VI swing-up | `lqr_at_operating_point` vs `DynamicProgrammingPlanner` |
| `labs/drone_ppo_learn_to_fly.ipynb` | C.1.4 Apprendre à voler | `Sys2Gym` bridge + PPO (stable-baselines3) |
| `labs/pendulum_dp_vs_ppo.ipynb` | Intro demo — DP vs RL | DP policy vs learned PPO policy on the same cost |

The two RL notebooks require the optional dependencies `gymnasium` and
`stable-baselines3` (training runs of ~10 minutes on the free Colab CPU tier).
