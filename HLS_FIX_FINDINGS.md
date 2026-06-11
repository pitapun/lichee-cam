# HLS Streaming Fix — LicheeRV Nano (CV1812H / CVI SDK)

## Final working state

- Device: 192.168.1.121 (licheervnano-3bff), also 192.168.100.195 (partial)
- HLS URL: `http://192.168.1.121:7778/hls/live.m3u8`
- Resolution: 1280x720 @ 15fps, 1500kbps, GOP=5
- Browser: hls.js (Hls.isSupported() check first; native only as Safari fallback)

---

## Root cause chain (all bugs, in order found)

### 1. NALU type detection using raw bytes (wrong)

**Symptom:** `stream_started` never set, no data written to FIFO, HLS 404.

**Root cause:** Used `d[4] & 0x1f` to detect H264 NALU type. Unreliable — start code
length varies (3 or 4 bytes) and the SDK byte layout differs from assumption.

**Fix:** Use `p->DataType.enH264EType` (SDK enum `H264E_NALU_TYPE_E`):
- `H264E_NALU_SPS` = SPS
- `H264E_NALU_PPS` = PPS
- `H264E_NALU_IDRSLICE` = IDR

---

### 2. drain_loop thread — GetStream always BUF_EMPTY (0xc0078012)

**Symptom:** Initial SPS/PPS/IDR written correctly (live0.ts, live1.ts appear), then all
subsequent `GetStream` calls return `CVI_ERR_VENC_BUF_EMPTY`. Segments freeze.

**Root cause:** `drain_loop` ran as a background thread continuously polling `GetStream`.
`CVI_VENC_GetStream` returns BUF_EMPTY immediately if no frame is currently in the VENC
pipeline — it does NOT block waiting for future `SendFrame` calls. The thread polls
independently of `SendFrame`, so it almost always finds an empty buffer.

**Fix:** Remove `drain_loop` thread entirely. Call `drain()` synchronously inside `send()`
right after `CVI_VENC_SendFrame` returns success — same pattern as `H264Recorder`.

```
send() {
    CVI_VENC_SendFrame(chn, frame, 100);  // inject frame
    drain(500);                            // immediately retrieve output
}
```

---

### 3. FIFO blocking write — D-state (uninterruptible sleep)

**Symptom:** `stream_yolo` process enters D-state (uninterruptible sleep in `write()`).
Can only be killed by reboot.

**Root cause:** When ffmpeg died (or was restarted), the FIFO pipe buffer filled up.
The previous code removed `O_NONBLOCK` from the FIFO open, enabling blocking writes.
A blocking `write()` to a full FIFO blocks forever if there is no reader.

**Fix:** Keep `O_NONBLOCK` on the FIFO file descriptor throughout. In `fifo_write()`:
- On `EAGAIN`/`EWOULDBLOCK`: drain stale data with `read()`, reset `stream_started`
- This forces the next IDR before resuming (clean re-sync for ffmpeg)

---

### 4. captureRaw path skipped HLS send entirely

**Symptom:** HLS segments freeze after the first ~15 frames when ambient light is low
(captureRaw path active, bypassing the normal frame path).

**Root cause:** The `captureRaw` code path used `continue` to skip all processing
after raw capture, including the HLS `send()` call.

**Fix:** Add HLS send in the captureRaw path using `last_disp` (last regular display
frame stored as a member variable):
```cpp
if (hls.is_active() && last_disp.stVFrame.u32Width > 0)
    hls.send(&last_disp);
```
Rate-limited to 25fps to avoid flooding VENC.

---

### 5. CHN0/CHN1 hardware conflict

**Symptom:** After an event recording (CHN1 at 2560x1440), HLS (CHN0 at 1280x720)
stops working. `SendFrame` returns `0x1` indefinitely.

**Root cause:** CV1812H VENC hardware cannot run CHN0 and CHN1 simultaneously.
`H264Recorder::stop()` called `CVI_VENC_StopRecvFrame(1)` which corrupted shared VENC
state, causing CHN0 to return `CVI_TRUE (1)` — actually "frame queued async", not failure.

**Fix (multi-part):**
1. Treat `ret == CVI_TRUE (1)` as success (same as `CVI_SUCCESS (0)`).
2. `pause_venc()` before event recording: `venc_active=false`, 60ms sleep, destroy CHN0.
3. `resume_venc()` after event: recreate CHN0 fresh, set `idr_pending=true`.
4. Guard `drain_loop` (removed in fix #2) and `send()` with `venc_active` flag.

---

### 6. Display rotation (camera mounted upside-down)

**Symptom:** HLS and MJPEG streams appear upside-down in browser.

**Root cause:** Camera is physically mounted inverted. CSS `transform:rotate(180deg)` in
`#hlsPlayer` rule was present but not reliably applied (browser cache returning old CSS).

**Fix:** Apply rotation via inline JavaScript in `setStream()`, which overrides CSS cache:
```js
const CAM_ROTATE = 'rotate(180deg)';
// in HLS mode:  hlsEl.style.transform = CAM_ROTATE
// in MJPEG mode: mjpegEl.style.transform = CAM_ROTATE
```

AI detection boxes are pre-rendered into MJPEG frames by `stream_yolo`, so rotating the
`<img>` element rotates the boxes correctly. HLS has no detection overlay.

---

## Final HlsStreamer design (main.cpp)

Key points:
- No background drain thread
- `send()` calls `drain(500ms)` synchronously after each `SendFrame`
- FIFO opened `O_RDWR | O_NONBLOCK` (keeps pipe alive, never blocks on write)
- Non-blocking `fifo_write()` drains on EAGAIN (resets stream for clean IDR sync)
- IDR requested on `start()` and on each `resume_venc()`
- `pause_venc()` / `resume_venc()` used around event recording to serialize CHN0/CHN1

## ffmpeg command (sidecar.py)

```
ffmpeg -fflags nobuffer -flags low_delay
       -f h264 -i /tmp/hls_feed.h264
       -c copy -f hls
       -hls_time 2 -hls_list_size 5
       -hls_flags delete_segments+append_list+omit_endlist
       /tmp/hls/live.m3u8
```

## Deploy recipe

```bash
# sidecar-only:
scp -o ProxyJump=pi5-4g sidecar.py root@192.168.1.121:/root/sidecar.py
ssh -o ProxyJump=pi5-4g root@192.168.1.121 'killall stream_yolo 2>/dev/null; fuser -k 7778/tcp 2>/dev/null; sleep 2; /etc/init.d/S98ninti_sidecar start'

# binary + sidecar (requires reboot for clean VPSS state):
scp -o ProxyJump=pi5-4g build-yolo/bin/stream_yolo root@192.168.1.121:/root/stream_yolo.new
scp -o ProxyJump=pi5-4g sidecar.py root@192.168.1.121:/root/sidecar.py
ssh -o ProxyJump=pi5-4g root@192.168.1.121 'mv /root/stream_yolo.new /root/stream_yolo && reboot'
```
