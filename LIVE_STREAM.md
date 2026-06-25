# Live Stream

The primary live stream is WebSocket H264, not HLS.

## Endpoint

Live endpoint:

```text
ws://<device>:7778/ws/live
```

The web UI uses `JMuxer` to feed received H264 access units into a `<video>`
element.

Required files on device:

- `/root/index.html`
- `/root/ha-stream.html`
- `/root/jmuxer.min.js`
- `/root/sidecar.py`

## WSFS Packet Format

Each WebSocket binary message carries one WSFS frame:

```text
offset  size  field
0       4     magic: "WSFS"
4       1     flags, bit 0 = keyframe
5       1     reserved
6       2     header size, currently 64
8       4     sequence number, uint32 big-endian
12      8     timestamp, microseconds, uint64 big-endian
20      4     width, uint32 big-endian
24      4     height, uint32 big-endian
28      4     payload length, uint32 big-endian
32      32    reserved
64      N     raw H264 payload
```

The browser parses WSFS, extracts the H264 payload, and feeds it to JMuxer.

## Configuration

Live stream settings:

```json
{
  "ws_width": 854,
  "ws_height": 480,
  "ws_fps": 10
}
```

The sidecar maps these to the historical C++ env names:

- `YOLO_HLS_W`
- `YOLO_HLS_H`
- `YOLO_HLS_FPS`

Those names are legacy; they now control WS live.

## Verification

Handshake test:

```bash
ssh root@$IP 'python3 - <<'"'"'PY'"'"'
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
```

Expected:

```text
HTTP/1.1 101 Switching Protocols
```

Asset test:

```bash
curl -s -o /dev/null -w "%{http_code}\n" http://$IP:7778/jmuxer.min.js
```

Expected:

```text
200
```

## Common Failures

`JMuxer is not defined` means `/jmuxer.min.js` was not deployed or is returning
404.

Fix by deploying all web/runtime files:

```bash
scripts/deploy-web.sh root@192.168.1.121
scripts/deploy-web.sh --jump pi5-4g root@192.168.100.195
```

`/ws/live` does not return `101` means the sidecar is not serving the WS handler
or port `7778` is stale.

Check:

```bash
ps w | grep -E "sidecar|stream_yolo"
tail -80 /tmp/ninti_sidecar.log
```

