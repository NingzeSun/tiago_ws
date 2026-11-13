# Sourced by the container scripts. Picks the container engine: podman when installed (the lab
# desktop L2), Docker otherwise (RUMI). TIAGO_ENGINE=docker|podman overrides the choice.
if [ -z "${TIAGO_ENGINE:-}" ]; then
    if command -v podman >/dev/null 2>&1; then TIAGO_ENGINE=podman; else TIAGO_ENGINE=docker; fi
fi
# Rootless podman maps container root to the host user, so the PAL `user` (host uid) needs
# --userns=keep-id and container root cannot open devices the host user can.
TIAGO_ROOTLESS=0
if [ "$TIAGO_ENGINE" = podman ] && [ "$(podman info --format '{{.Host.Security.Rootless}}' 2>/dev/null)" = true ]; then
    TIAGO_ROOTLESS=1
fi
ctr() { "$TIAGO_ENGINE" "$@"; }
