#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/deploy-web.sh [--jump HOST] [--reboot] root@HOST

Deploys the sidecar web/runtime files that the WS live UI needs:
  - sidecar.py
  - index.html
  - ha-stream.html
  - jmuxer.min.js

The script also migrates legacy hls_* config keys to ws_* and verifies that:
  - /api/config exposes ws_* keys
  - /jmuxer.min.js is served
  - /ws/live accepts a WebSocket upgrade

Examples:
  scripts/deploy-web.sh root@192.168.1.121
  scripts/deploy-web.sh --jump pi5-4g --reboot root@192.168.100.195
EOF
}

jump_host=""
do_reboot=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --jump|-J)
      [[ $# -ge 2 ]] || { echo "missing value for $1" >&2; exit 2; }
      jump_host="$2"
      shift 2
      ;;
    --reboot)
      do_reboot=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    -*)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      break
      ;;
  esac
done

[[ $# -eq 1 ]] || { usage >&2; exit 2; }
target="$1"

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
required=(
  sidecar.py
  index.html
  ha-stream.html
  jmuxer.min.js
)

for f in "${required[@]}"; do
  [[ -s "$repo_dir/$f" ]] || { echo "missing required file: $f" >&2; exit 1; }
done

ssh_opts=(-o ConnectTimeout=8)
scp_opts=(-O)
if [[ -n "$jump_host" ]]; then
  ssh_opts+=(-J "$jump_host")
  scp_opts+=(-o "ProxyJump=$jump_host")
fi

echo "[deploy] target=$target jump=${jump_host:-none} reboot=$do_reboot"
echo "[deploy] copying web/runtime files"
scp "${scp_opts[@]}" \
  "$repo_dir/sidecar.py" \
  "$repo_dir/index.html" \
  "$repo_dir/ha-stream.html" \
  "$repo_dir/jmuxer.min.js" \
  "$target:/root/"

echo "[deploy] migrating config to ws_* keys"
ssh "${ssh_opts[@]}" "$target" 'python3 - <<'"'"'PY'"'"'
import json

p = "/root/ninti_config.json"
with open(p) as f:
    cfg = json.load(f)

ww = int(cfg.get("ws_width") or cfg.get("hls_width") or cfg.get("sensor_width") or 1280)
wh = int(cfg.get("ws_height") or cfg.get("hls_height") or cfg.get("sensor_height") or 720)
wf = int(cfg.get("ws_fps") or cfg.get("hls_fps") or 5)

cfg["ws_width"] = ww
cfg["ws_height"] = wh
cfg["ws_fps"] = wf
for key in ("hls_width", "hls_height", "hls_fps"):
    cfg.pop(key, None)

with open(p, "w") as f:
    json.dump(cfg, f, indent=2)

print({k: cfg.get(k) for k in (
    "sensor_width", "sensor_height", "ws_width", "ws_height",
    "ws_fps", "record_fps", "target_fps"
)})
PY'

if [[ "$do_reboot" -eq 1 ]]; then
  echo "[deploy] rebooting target"
  ssh "${ssh_opts[@]}" "$target" 'sync; reboot' || true
  echo "[deploy] waiting for ssh"
  for _ in $(seq 1 24); do
    if ssh "${ssh_opts[@]}" "$target" 'echo up' >/dev/null 2>&1; then
      break
    fi
    sleep 5
  done
fi

echo "[deploy] verifying"
ssh "${ssh_opts[@]}" "$target" 'set -e
echo "--- process ---"
ps w | grep -E "sidecar|stream_yolo" | grep -v grep || true
echo "--- config ---"
curl -s --max-time 3 http://127.0.0.1:7778/api/config | python3 -c '"'"'
import json, sys
cfg = json.load(sys.stdin)
print({k: cfg.get(k) for k in ("ws_width", "ws_height", "ws_fps", "sensor_width", "sensor_height", "record_fps", "target_fps")})
print("legacy_hls_keys", [k for k in cfg if k.startswith("hls_")])
'"'"'
echo "--- jmuxer ---"
wget -S -O /dev/null http://127.0.0.1:7778/jmuxer.min.js 2>&1 | head -6
echo "--- ws live ---"
python3 - <<'"'"'PY'"'"'
import base64, os, socket

host = "127.0.0.1"
port = 7778
key = base64.b64encode(os.urandom(16)).decode()
s = socket.create_connection((host, port), timeout=5)
s.sendall((
    f"GET /ws/live HTTP/1.1\r\n"
    f"Host: {host}:{port}\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    f"Sec-WebSocket-Key: {key}\r\n"
    "Sec-WebSocket-Version: 13\r\n\r\n"
).encode())
resp = b""
while b"\r\n\r\n" not in resp:
    resp += s.recv(4096)
print(resp.decode(errors="ignore").split("\r\n")[0])
s.close()
PY'

echo "[deploy] done"
