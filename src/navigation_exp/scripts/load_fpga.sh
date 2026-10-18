#!/bin/bash

set -euo pipefail

ACCEL_NAME="gradient_accel"
MAX_WAIT_SECONDS=60

if [ "$(id -u)" -ne 0 ]; then
    exec sudo "$0" "$@"
fi

loaded_name() {
    xmutil listapps | awk '$1 ~ /^[0-9]+$/ && $(NF-1) ~ /->/ { print $NF; exit }'
}

ip link set eth1 up
sleep 5
IS_UP=$(ip link show eth1 | grep -c "state UP")
if [ "$IS_UP" -eq 0 ]; then
    echo "eth1 is not up" >&2
    exit 1
fi

waited=0
until xmutil listapps >/dev/null 2>&1; do
    if [ "$waited" -ge "$MAX_WAIT_SECONDS" ]; then
        echo "xmutil not ready after ${MAX_WAIT_SECONDS} sec" >&2
        exit 1
    fi
    sleep 2
    waited=$((waited + 2))
done

current="$(loaded_name)"

if [ "$current" = "$ACCEL_NAME" ]; then
    echo "$ACCEL_NAME is already loaded"
    exit 0
fi

if [ -n "$current" ]; then
    echo "Unload $current"
    xmutil unloadapp || true
fi

echo "Loading $ACCEL_NAME"
xmutil loadapp "$ACCEL_NAME" || true

if [ "$(loaded_name)" != "$ACCEL_NAME" ]; then
    echo "Failed to load $ACCEL_NAME" >&2
    exit 1
fi
echo "$ACCEL_NAME loaded"