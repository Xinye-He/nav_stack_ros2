#!/usr/bin/env bash
set -euo pipefail
MONITOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for pf in "$MONITOR_DIR"/cam*.pid; do
  [ -f "$pf" ] || continue
  pid="$(cat "$pf")"
  name="$(basename "$pf" .pid)"
  # Ignore stale or reused PIDs; never kill unrelated processes.
  if [[ "$pid" =~ ^[0-9]+$ ]] && [ "$pid" -gt 1 ] && [ -r "/proc/$pid/cmdline" ]; then
    cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline")"
    if [[ "$cmd" == *gst-launch-1.0* && "$cmd" == *"/$name"* ]]; then
      kill "$pid" 2>/dev/null || true
      for attempt in {1..20}; do
        if ! kill -0 "$pid" 2>/dev/null; then break; fi
        sleep 0.1
      done
    fi
  fi
  rm -f "$pf"
done
docker rm -f mediamtx 2>/dev/null || true
echo "监控推流已停止"
