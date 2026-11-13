#!/usr/bin/env bash
# Connect the Quest to THIS machine with adb (inside the tiago_dc container) and open the teleop page.
#
#   scripts/quest_connect.sh usb              Quest plugged into this machine by USB
#   scripts/quest_connect.sh enable-wifi      once, while plugged in: switch the Quest to Wi-Fi adb, print its IP
#   scripts/quest_connect.sh wifi <quest-ip>  no cable: Quest and this machine on the same network
#
# All modes end with `adb reverse tcp:<port> tcp:<port>` (the headset's localhost:<port> becomes
# this machine's teleop page) and open the page in the Quest Browser. USB needs a container
# created with TIAGO_USB=1 docker/run_ros.sh. The headset must have Developer Mode on and you must
# accept "Allow USB debugging" in it the first time.
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
NAME=${TIAGO_CONTAINER:-tiago_dc}
PORT=$(awk '/^quest:/{q=1} q && /http_port:/{print $2; exit}' "$WS/${TIAGO_DC_CONFIG_HOST:-config/sim.yaml}")
source "$WS/docker/engine.sh"
# adb needs write access to the USB device node: container root under Docker, the host user
# (`user`) under rootless podman.
ADB_USER=root
[ "$TIAGO_ROOTLESS" = 1 ] && ADB_USER=user
adb() { ctr exec -u "$ADB_USER" "$NAME" adb "$@"; }

case "${1:-}" in
    usb)
        adb start-server >/dev/null
        adb wait-for-usb-device
        ;;
    enable-wifi)
        adb start-server >/dev/null
        adb wait-for-usb-device
        adb tcpip 5555
        sleep 2
        ip=$(adb shell ip -f inet addr show wlan0 | awk '/inet /{sub(/\/.*/, "", $2); print $2}')
        echo "Quest Wi-Fi IP: ${ip:-unknown}. Unplug it and run: scripts/quest_connect.sh wifi $ip"
        exit 0
        ;;
    wifi)
        [ -n "${2:-}" ] || { echo "usage: $0 wifi <quest-ip>" >&2; exit 2; }
        adb start-server >/dev/null
        adb connect "$2:5555"
        adb -s "$2:5555" wait-for-device
        ;;
    *)
        sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
        exit 2
        ;;
esac

adb devices -l
adb reverse "tcp:$PORT" "tcp:$PORT"
if [ -f "$WS/data/quest_url.txt" ]; then
    url=$(cat "$WS/data/quest_url.txt")
    adb shell am start -a android.intent.action.VIEW -d "$url" com.oculus.browser >/dev/null
    echo "opened $url in the Quest Browser"
else
    echo "reverse ready; start the teleop stack, then open data/quest_url.txt's URL in the Quest Browser"
fi
