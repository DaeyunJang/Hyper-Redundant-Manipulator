#!/usr/bin/env bash
# AUTHOR: Daeyun - explicit Docker lifecycle for the native HRM system.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec /usr/bin/python3 "$repo_root/docker/manage.py" "$@"

