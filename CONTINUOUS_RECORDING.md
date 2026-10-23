# Continuous ("dash cam") recording

Rolling 10-minute clips, recorded 24/7, oldest deleted when the budget is hit.
Independent of the motion/YOLO event recorder described in
[EVENT_RECORDING.md](EVENT_RECORDING.md) — both can run at once.

## How it works

The recorder is a **tee on the live stream**, not a second encoder.

`stream_yolo` already encodes CHN0 (VPSS CHN1 -> VENC chn 0) to Annex-B H264 and
writes it to `/tmp/hls_feed.h264`, and `sidecar.py:_live_ws_reader_loop()` already
reads every byte of that FIFO to feed `/ws/live` — continuously, whether or not a
browser is attached. `_clip_write()` hangs off that same loop, so the only added
cost is one `write()` per FIFO chunk plus one `-c copy` remux per finished clip.

Clips are therefore at the **live stream's** resolution and bitrate (`ws_width` /
`ws_height` / `ws_fps`), not the event recorder's. Raising the live resolution
raises clip quality and cost together.

Running the CHN1 record encoder continuously instead was rejected: it is a second
1080p encode on a single-core SoC that already sits near load 3, and CHN1 is owned
by the event recorder, which creates and destroys it per event.

### Rotation

CHN0 emits SPS+PPS+IDR about every 2 s. Once `clips_secs` has elapsed, the writer
cuts at the **first SPS start code** in the next chunk, so every clip opens with
SPS/PPS/IDR and decodes on its own. Cuts therefore land within a couple of seconds
of the target length, never exactly on it.

### Remux

Annex-B carries no timestamps, so a clip remuxed at the *configured* fps plays at
the wrong speed whenever the device misses its frame rate — the bug that already
bit the event path once. Instead the writer counts VCL slice NALs (types 1 and 5)
as it goes and divides by the clip's real wall-clock span. One background worker
runs `ffmpeg -c copy` per finished clip and deletes the `.h264` on success.

A `.h264` left behind by a crash is picked up at startup by
`_clip_startup_recover()` and remuxed from its name stamp and mtime.

## Storage

Two **explicit byte budgets**, because a percentage-of-filesystem watermark cannot
share a card between clips and events — each would grow into the other's space and
they would take turns deleting each other.

| setting | default | what it bounds |
|---|---|---|
| `clips_max_gb` | 20.0 | `/root/ninti_clips` |
| `events_max_gb` | 6.0 | `/root/ninti_events` |
| `CLIP_MIN_FREE_BYTES` | 2.5 GB | free space clips refuse to eat into |
| `EVENT_STORAGE_MAX_USED_PCT` | 95 / 92 | emergency valve only |

`CLIP_MIN_FREE_BYTES` is deliberately **larger** than the free space the emergency
valve implies, so clips always yield first and the valve stays closed.

Events are pruned oldest-first by the save epoch in the dir name
(`event_00001_save_1785821317`), never by mtime — reclaiming a duplicate
`event.h264` rewrites the parent dir's mtime and would otherwise send the prune
after the wrong events.

### Duplicate `event.h264`

An event used to keep both `event.h264` and its `event.mp4` stream copy. The mp4
is now the only survivor; `_cleanup_event_storage()` also sweeps up the pairs left
by older builds. On 192.168.100.196 that reclaimed 9.67 GB on first run.

## Config

```json
{
  "clips_enabled": true,
  "clips_secs": 600,
  "clips_max_gb": 20.0,
  "events_max_gb": 8.5
}
```

`clips_secs` is clamped to 30..3600, `clips_max_gb` and `events_max_gb` to
0.5..64. None of these restart `stream_yolo`; setting `clips_enabled` false
finalises the clip in progress and stops.

## API

- `GET /api/clips` — `{enabled, secs, max_gb, used_gb, recording, pending_remux, clips[]}`
  where each clip is `{name, url, size, mtime, started, duration, end, fps}`, newest first.
- `GET /clip-video/<name>.mp4` — the clip, with `Range` support so the browser can seek.
- `GET /api/clip-marks?since=<epoch>` — `[{t, cat, d}]`, the detection events that fall in
  the recorded window, for the timeline's activity marks. Tail-reads `events.jsonl`; parsing
  the whole 2.4 MB index timed the request out on the device.

### Clip durations

`clips.jsonl` in `CLIP_DIR` records `{name, started, duration, frames, fps}` at remux, because
that is the only moment the true duration is known. A clip with no index entry (one recorded
before the index existed) falls back to the next clip's start time, which is correct while
clips are contiguous — they are, since the next clip opens in the same call that closes the
previous one — and to `clips_secs` for the newest. Pruning rewrites the index.

## The Clips tab

A recording timeline, not a file list. The clips are separate mp4s but the user thinks in
wall-clock time, so the UI maps an instant to `(clip, offset)` and back:

- Vertical timeline on the right, newest at the top, with recorded-coverage bars, time
  gridlines and detection marks (orange = person, blue = vehicle). Click or drag to seek to
  that instant; `+`/`-` zoom from about 10 s/px to 1 s/px.
- The wall-clock time of the frame on screen is overlaid on the video, computed from the
  clip's start plus `currentTime` — no need to burn a timestamp into the encode.
- `ended` rolls straight into the next segment, so an hour plays as one recording.
- `«10s` / `10s»`, play/pause, 1x-2x-4x-8x, **Latest**, fullscreen.
- **List** toggles back to the plain newest-first list; clicking a row seeks the timeline.

- Keyboard: `<-`/`->` (or `v`/`^`) +-10s, with Shift +-1 min and Ctrl/Cmd +-10 min; `space`
  play/pause; `,`/`.` frame step; `[`/`]` set the export range; `Home`/`End` oldest/latest;
  `+`/`-` zoom; `f` fullscreen. Ignored while a form field has focus.

The tab pauses playback when you navigate away — a 10-minute mp4 keeps downloading otherwise.

### Storage readout

The header shows `clips X/Y GB - events X/Y GB - card N GB free of T (~Hh more)`. `hours_left`
divides the remaining room (clip budget or the free-space floor, whichever binds first) by the
measured bytes-per-second of footage, so it tracks the real scene bitrate rather than a guess.

`storage` in `/api/clips` carries the same figures. The events-dir size is cached
(`EVENTS_BYTES_TTL`, 180 s) and refreshed off the request thread — walking ~2000 event
directories inline timed `/api/clips` out.

## Export

`GET /api/export.mp4?from=<epoch>&to=<epoch>&tz=<local offset minutes>` streams the range as a
single mp4. It is **never written to the card**: the card is already split between clips and
events and has no room for a spare copy, so ffmpeg's stdout is piped straight to the client.

    ffmpeg -f concat -safe 0 -i <list> -ss <off> -t <dur> -c copy \
           -avoid_negative_ts make_zero \
           -movflags frag_keyframe+empty_moov+default_base_moof -f mp4 pipe:1

`-ss`/`-t` must be **output** options. As input options the concat demuxer honours `-ss` and
silently ignores `-t` (asking for 40 s gave 67.6 s); on the output side both apply, and the
first frame still comes out an IDR, so the clip does not open on garbage. Measured about 10x
realtime on the device.

- One export at a time (`429` otherwise) — it competes with the recorder for the card.
- Range must be 2 s to `EXPORT_MAX_SECS` (1 hour), else `400`.
- Runs under `nice -n 19`.
- Filename is `ninti_<YYYYmmdd_HHMM>_<N>min.mp4` in the **browser's** timezone, from `tz`.

In the UI: scrub to the start, press `[ In`, scrub to the end, press `Out ]`, then **Export**
(the button shows the length and estimated size). The selection is shaded on the timeline.

## Reboot forensics

Every log this firmware writes lives in `/tmp`, which is tmpfs, so a reboot destroys the
evidence for why it rebooted — including the watchdog's own reason. `/root/ninti_reboot.log`
is on the SD card and records `BOOT` / `SIDECAR_START` / `WATCHDOG_REBOOT` / `MANUAL_REBOOT`
with uptime, load, free memory, stream_yolo's RSS and state, free disk, and the last 15 lines
of `stream_yolo.log`. It fsyncs before returning, because `reboot -f` does not sync and would
otherwise throw away the line describing itself. Capped at 400 lines.

**A `BOOT` line with no `WATCHDOG_REBOOT` or `MANUAL_REBOOT` before it means nothing in the
software asked for the reboot** — power, brownout or the hardware watchdog.

## Sizing

Measured on 192.168.100.196 at 854x480@10: **2069 kbps = 886 MB/h = ~155 MB per
10-minute clip = ~21 GB/day**. (The `/ws/live` figure is slightly lower, 1799 kbps,
because the WS path drops frames for a slow client and the recorder does not.)

Divide the budget by 886 MB/h for hours retained: 20 GB is a bit under a day.

**SD card wear.** ~21 GB/day is ~7.7 TB/year of writes. A generic card will not
last; use a high-endurance / dash-cam rated one.

## Tests

```
python3 tests/test_clip_recorder.py     # segmentation, fps recovery, clip budget
python3 tests/test_event_retention.py   # h264 reclaim, event budget, prune order
```

Both run on the host with no device and no ffmpeg.
