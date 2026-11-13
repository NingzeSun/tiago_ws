#!/usr/bin/env bash
# Build tiago-dc:dev on top of the PAL development image.
set -euo pipefail
cd "$(dirname "$0")"
source ./engine.sh
if ! ctr image inspect development-tiago-pro-30:alum-edge >/dev/null 2>&1; then
    echo "Missing development-tiago-pro-30:alum-edge; run 'bash uwaterloo-air/uwaterloo-air-build-docker.sh' from the workspace first" >&2
    exit 1
fi
ctr build -t tiago-dc:dev -f Dockerfile.dev .
