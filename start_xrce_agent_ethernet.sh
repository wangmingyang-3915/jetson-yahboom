#!/usr/bin/env bash
set -u

agent=/home/jetson/.local/bin/MicroXRCEAgent
agent_pid=

cleanup() {
    trap - EXIT INT TERM
    if [[ -n "$agent_pid" ]]; then
        kill "$agent_pid" 2>/dev/null || true
        wait "$agent_pid" 2>/dev/null || true
    fi
    exit 0
}

trap cleanup EXIT INT TERM

export LD_LIBRARY_PATH="/home/jetson/.local/lib:${LD_LIBRARY_PATH:-}"

while true; do
    echo "启动 Ethernet Micro XRCE-DDS Agent：UDP 0.0.0.0:8888"
    "$agent" udp4 -p 8888 &
    agent_pid=$!
    wait "$agent_pid" || true
    agent_pid=

    echo "Ethernet Micro XRCE-DDS Agent 已退出，1 秒后重启" >&2
    sleep 1
done
