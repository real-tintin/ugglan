#!/bin/sh
# Setup enviroment for project. Source
# and run from the repo root.
#

PY_VENV_ROOT="./venv"
PY_VENV_REQ="./requirements.txt"

# Setup python using venv.
python -m venv "${PY_VENV_ROOT}"
source "${PY_VENV_ROOT}/Scripts/activate"

pip install --upgrade pip
pip install -r "${PY_VENV_REQ}"

# Init and update submodules
git submodule init
git submodule update
