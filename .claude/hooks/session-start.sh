#!/bin/bash
# Claude Code cloud sessions: install the Full tier of install.md the way CI's
# regression job does, so the local gate (ruff, pytest, the demo and notebook
# checks, the docs build) runs in the container. A local session skips it.
set -euo pipefail

if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

cd "${CLAUDE_PROJECT_DIR:-$(git rev-parse --show-toplevel)}"

# The dot binary, which pip cannot install (install.md).
if ! command -v dot >/dev/null 2>&1; then
  SUDO=""
  [ "$(id -u)" -eq 0 ] || SUDO=sudo
  $SUDO apt-get update -q
  $SUDO apt-get install -y -q --no-install-recommends graphviz
fi

# The libraries: the CI regression extras plus the docs build. Ipopt stays
# conda-only (install.md); trajectory optimization runs on SciPy without it.
python -m pip install -q -e ".[dev,jax,visualization,plotting,diagrams,rl,symbolic,docs]"

# Headless plots for the demo and notebook checks.
if [ -n "${CLAUDE_ENV_FILE:-}" ]; then
  echo 'export MPLBACKEND=Agg' >> "$CLAUDE_ENV_FILE"
fi
