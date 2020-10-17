#!/bin/sh
# Setup enviroment for project. Source
# and run from the repo root.
#

REPO_ROOT="$(pwd)"

PY_VENV_ROOT="./venv"
PY_VENV_REQ="./requirements.txt"

# Setup python using venv.
python -m venv "${PY_VENV_ROOT}"
source "${PY_VENV_ROOT}/Scripts/activate"

pip install --upgrade pip
pip install -r "${PY_VENV_REQ}"

# Add python src roots to PYTHONPATH.
DATA_LOG_SRC="${REPO_ROOT}/analysis/data_log"
MULTI_BODY_SRC="${REPO_ROOT}/analysis/multi_body"
STATE_ESTIMATION_SRC="${REPO_ROOT}/analysis/state_estimation"

PYTHONPATH="${DATA_LOG_SRC}:${MULTI_BODY_SRC}:${STATE_ESTIMATION_SRC}"

# Update python aliases to include PYTHONPATH.
alias py="PYTHONPATH=${PYTHONPATH} py"
alias python="PYTHONPATH=${PYTHONPATH} python"

# Init and update submodules.
git submodule init
git submodule update
