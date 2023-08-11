#!/usr/bin/env bash

SLIM_SETUP=false

VENV_ROOT="./venv"

usage() {
  echo "\
Usage: source ./setup [OPTIONS]

Use to setup local development environment.

Options:
    --slim      skip some steps in order to speed up the setup, useful if recently setup
  " 1>&2
}

if [[ $# -gt 0 ]]; then
    case $1 in
        --slim)
            SLIM_SETUP=true
            ;;
        --help)
            usage
            return
            ;;
        *)
            usage
            return
            ;;
    esac
fi

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

activate_venv() {
  if is_windows; then
      source "${VENV_ROOT}/Scripts/activate"
  else
      source "${VENV_ROOT}/bin/activate"
  fi
}

setup_submodules() {
    git submodule update --init --recursive
}

setup_python() {
    python3 -m venv "${VENV_ROOT}"
    activate_venv

    python3 -m pip install --upgrade pip
    python3 -m pip install -e ./tools
}

setup_python_slim() {
  activate_venv
}

setup_env_vars() {
  export DOCKER_BUILDKIT=1
}

if [[ "${SLIM_SETUP}" == "true" ]]; then
    echo "Skipping some steps in order to speed up the setup"

    setup_python_slim
    setup_env_vars
else
    setup_submodules
    setup_python
    setup_env_vars
fi
