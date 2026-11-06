#!/usr/bin/env bash
# Clothoid-A 2026 Dev Docker Runner
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker/docker-compose.yml"

# 1. X11 GUI display authorization
if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 || true
fi

export LOCAL_USER_ID="$(id -u)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# 2. Check options
BUILD_FLAG=""
if [[ "$1" == "--build" || "$1" == "-b" ]]; then
    BUILD_FLAG="--build"
    echo "[Clothoid-A] Building Docker image..."
    docker compose -f "${COMPOSE_FILE}" build
    shift
fi

# 3. Check if clothoid-dev container is already running
if docker ps --format '{{.Names}}' | grep -q "^clothoid-dev$"; then
    echo "[Clothoid-A] Container 'clothoid-dev' is already running. Attaching shell..."
    docker exec -it -u user clothoid-dev bash
    exit 0
fi

# 4. Run development container
echo "[Clothoid-A] Starting ROS 2 Humble development container..."
echo "             - Workspace: ~/Clothoid-A/ros2_ws"
echo "             - User ID:   ${LOCAL_USER_ID}"
echo "             - GPU:       Enabled (NVIDIA)"
echo "--------------------------------------------------------"

if [ $# -eq 0 ]; then
    docker compose -f "${COMPOSE_FILE}" run --rm dev /bin/bash
else
    docker compose -f "${COMPOSE_FILE}" run --rm dev "$@"
fi
