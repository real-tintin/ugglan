#!/bin/sh
# Setup enviroment for project. Source
# and run from the repo root.
#

# Setup python using venv.
PY_VENV_ROOT="./venv"

python -m venv "${PY_VENV_ROOT}"

is_windows() {
  case "$(uname -sr)" in
    CYGWIN*|MINGW*|MINGW32*|MSYS*)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

if is_windows; then
    source "${PY_VENV_ROOT}/Scripts/activate"
else
    source "${PY_VENV_ROOT}/bin/activate"
fi

pip install --upgrade pip
pip install ./tools

# Init and update submodules.
git submodule init
git submodule update

# Use docker buildx/buildkit by default.
export DOCKER_BUILDKIT=1
