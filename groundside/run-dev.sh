#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$ROOT_DIR/backend"
FRONTEND_DIR="$ROOT_DIR/frontend"

BLUE=$'\033[1;34m'
GREEN=$'\033[1;32m'
YELLOW=$'\033[1;33m'
RESET=$'\033[0m'

pick_python() {
  if [[ -n "${PYTHON:-}" ]]; then
    printf '%s\n' "$PYTHON"
    return
  fi

  local candidates=(
    "$ROOT_DIR/venv/bin/python"
    "$ROOT_DIR/../venv/bin/python"
  )

  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -x "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return
    fi
  done

  if command -v python3 >/dev/null 2>&1; then
    printf '%s\n' "python3"
    return
  fi

  if command -v python >/dev/null 2>&1; then
    printf '%s\n' "python"
    return
  fi

  printf 'No Python interpreter found.\n' >&2
  exit 1
}

prefix_logs() {
  local label="$1"
  local color="$2"

  awk -v label="$label" -v color="$color" -v reset="$RESET" '
    {
      printf "%s[%s]%s %s\n", color, label, reset, $0
      fflush()
    }
  '
}

PYTHON_BIN="$(pick_python)"

if ! command -v pnpm >/dev/null 2>&1; then
  printf 'pnpm is required but was not found in PATH.\n' >&2
  exit 1
fi

printf '%s[info]%s backend:  http://127.0.0.1:5000\n' "$YELLOW" "$RESET"
printf '%s[info]%s frontend: http://127.0.0.1:5173\n' "$YELLOW" "$RESET"
printf '%s[info]%s press Ctrl+C to stop both services\n' "$YELLOW" "$RESET"

cleanup() {
  local exit_code=$?
  trap - INT TERM EXIT

  if [[ -n "${BACKEND_PID:-}" ]]; then
    kill "$BACKEND_PID" 2>/dev/null || true
  fi

  if [[ -n "${FRONTEND_PID:-}" ]]; then
    kill "$FRONTEND_PID" 2>/dev/null || true
  fi

  wait 2>/dev/null || true
  exit "$exit_code"
}

trap cleanup INT TERM EXIT

(
  cd "$BACKEND_DIR"
  "$PYTHON_BIN" app.py 2>&1 | prefix_logs "backend" "$BLUE"
) &
BACKEND_PID=$!

(
  cd "$FRONTEND_DIR"
  pnpm dev 2>&1 | prefix_logs "frontend" "$GREEN"
) &
FRONTEND_PID=$!

wait -n "$BACKEND_PID" "$FRONTEND_PID"
