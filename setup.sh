#!/bin/sh
# Setup enviroment for project. Source
# and run from the repo root.
#

PYENV_ROOT="./pyenv"
PYENV_REQ="./requirements.txt"

# Setup python virtual env.
python -m venv "${PYENV_ROOT}"
source "${PYENV_ROOT}/Scripts/activate"

pip install --upgrade pip
pip install -r "${PYENV_REQ}"
