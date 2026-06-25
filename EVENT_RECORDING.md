# Event Recording Policy

This document captures the intended sidecar event lifecycle. Keep this in sync
with `sidecar.py` when changing tracking or recording stop logic.

## Tracking

Detections arrive at `sidecar.py` over UDP port `5005`.

Each detection is matched to an existing track by class, IoU, and center
distance:

- `TRACK_IOU_THRESHOLD = 0.20`
- `TRACK_CENTER_THRESHOLD = 0.25`
- `TRACK_CONFIRM_HITS = 2`

A track becomes confirmed after 2 hits. Recording starts when the track becomes
confirmed.

## Recording Start

When a track is confirmed:

- `_start_event_video()` starts the HD recorder.
- `min_record_until = now + MIN_RECORD_SECS`.
- `MIN_RECORD_SECS = 3.0`.

The 3 second minimum avoids saving clips that are too short to be useful.

## Recording Stop

While recording, a stationary object must not stop the clip.

Current policy:

- Keep recording while the object is still detected.
- Stop recording after the object has not been detected for 1 second.
- Stop recording at the hard cap even if the object is still visible.

Timeouts:

- `RECORD_LOST_TIMEOUT = 1.0`
- `MAX_RECORD_SECS = 12.0`

Important: do not reintroduce a recording-time still timeout. The old
`RECORD_STILL_TIMEOUT = 2.5` caused clips to end around 3 seconds when a person
stood still in frame.

## Non-Recording Expiry

For tracks that are not currently recording, the sidecar may expire tracks by
lost or still timeout:

- `TRACK_LOST_TIMEOUT = 2.5`
- `TRACK_STILL_TIMEOUT = 5.0`

This is separate from recording stop behavior.

## Stationary Suppression

Stationary suppression exists to avoid repeated events from the same object
after a still-timeout expiry.

Apply stationary suppression only when a track expires because it was still
while not recording.

Do not apply stationary suppression when a recording ends because:

- the object disappeared,
- the recording hit `MAX_RECORD_SECS`,
- the event was otherwise completed normally.

## Expected Behavior

If a person is detected for 6 seconds and then disappears, the saved event
should be roughly 7 seconds metadata duration, with an MP4 duration close to the
recorded H264 duration.

Example verification from 192.168.100.195:

- 6 second fake person detection
- metadata `duration_s = 6.86`
- MP4 duration `6.3`
- no `stationary suppress` log line

## Verification

After changing event logic:

1. Deploy `sidecar.py`.
2. Restart the sidecar cleanly.
3. Confirm `/ws/live` still upgrades.
4. Trigger a controlled fake detection for at least 6 seconds.
5. Confirm the saved event duration is longer than the 3 second minimum.
6. Confirm the sidecar log does not show `stationary suppress` for a normal
   object-left event.

