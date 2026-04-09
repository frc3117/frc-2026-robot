#!/usr/bin/env bash
set -euo pipefail
BASEDIR=$(dirname "$0")
cd "${BASEDIR}"
./.venv/bin/robotpy sim "$@"
