#!/bin/sh
# Setup enviroment for project. Source
# and run from the repo root.
#

REPO_ROOT="$(pwd)"

# Setup python using venv.
PY_VENV_ROOT="./venv"
PY_VENV_REQ="./tools/requirements.txt"

python -m venv "${PY_VENV_ROOT}"

if [ "$(uname)" == "Linux" ]; then
    source "${PY_VENV_ROOT}/bin/activate"
else # Assume windows
    source "${PY_VENV_ROOT}/Scripts/activate"
fi

pip install --upgrade pip
pip install -r "${PY_VENV_REQ}"

# Init and update submodules.
git submodule init
git submodule update

# Use docker buildx/buildkit by default.
export DOCKER_BUILDKIT=1
