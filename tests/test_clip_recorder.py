#!/usr/bin/env python3
"""Segmentation + retention checks for the continuous ("dash cam") recorder.

Run from the repo root:  python3 tests/test_clip_recorder.py

These cover the parts that are painful to debug on the device: the cut has to
land on an SPS, no byte may be lost or duplicated at a boundary, and the fps
recovered from slice-count / wall-clock has to match reality or every clip
plays back at the wrong speed.
"""
import os, random, shutil, sys, tempfile

TD = tempfile.mkdtemp(prefix='cliptest-')
os.environ['NINTI_CONFIG'] = TD + '/cfg.json'
os.environ['NINTI_EVENT_DIR'] = TD + '/events'
os.environ['NINTI_CLIP_DIR'] = TD + '/clips'
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import sidecar as S

SC = b'\x00\x00\x00\x01'
MB = 1024 * 1024
fails = []


def nal(typ, n=40):
    return SC + bytes([typ]) + bytes((i * 7 + typ) % 251 + 1 for i in range(n))


def make_stream(secs, fps=10):
    """SPS+PPS+IDR every 2s, plain slices in between — what CHN0 emits."""
    out = bytearray()
    for f in range(secs * fps):
        if f % (2 * fps) == 0:
            out += nal(7, 20) + nal(8, 8) + nal(5, 300)
        else:
            out += nal(1, 120)
    return bytes(out), secs * fps


STREAM, NFRAMES = make_stream(300)
S.cfg['clips_enabled'] = True
S.cfg['clips_secs'] = 30          # CLIP_MIN_SECS floor; production runs 600
S.cfg['clips_max_gb'] = 64.0

# Fake clock so 300s of video costs no wall time, and capture what would have
# been queued for remux instead of shelling out to ffmpeg.
fake = [1_700_000_000.0]
real_time = S.time.time
S.time.time = lambda: fake[0]
remuxed = []
S._clip_remux_q.put = lambda item: remuxed.append(item)

random.seed(7)
bps = len(STREAM) / 300.0
pos = 0
while pos < len(STREAM):
    chunk = STREAM[pos:pos + random.randint(1, int(bps * 1.2))]  # <=1.2s, like a FIFO read
    pos += len(chunk)
    fake[0] += len(chunk) / bps
    S._clip_write(chunk)
with S._clip_lock:
    S._clip_close_locked(fake[0])
S.time.time = real_time

files = sorted(f for f in os.listdir(TD + '/clips') if f.endswith('.h264'))
print('clips produced: %d' % len(files))

# 1. no byte lost or duplicated across the cuts
cat = b''.join(open(os.path.join(TD, 'clips', f), 'rb').read() for f in files)
if cat != STREAM:
    fails.append('concat mismatch: %d bytes vs %d' % (len(cat), len(STREAM)))

# 2. every clip is independently decodable, i.e. opens on an SPS
for f in files:
    d = open(os.path.join(TD, 'clips', f), 'rb').read()
    off = 4 if d[:4] == SC else (3 if d[:3] == b'\x00\x00\x01' else -1)
    if off < 0 or (d[off] & 0x1f) != 7:
        fails.append('%s does not start on an SPS' % f)

# 3. per-clip slice counts match the file, and the fps they imply is the truth
total = 0
for path, frames, span in remuxed:
    actual = len(S._h264_scan(open(path, 'rb').read(), (1, 5)))
    if frames != actual:
        fails.append('%s counted %d slices, file has %d'
                     % (os.path.basename(path), frames, actual))
    total += frames
    fps = frames / span
    print('  %s frames=%d span=%.1fs -> %.2f fps' % (os.path.basename(path), frames, span, fps))
    if not 9.0 <= fps <= 11.0:
        fails.append('%s recovered %.2f fps from a true 10 fps' % (os.path.basename(path), fps))
if total != NFRAMES:
    fails.append('total frames %d != %d' % (total, NFRAMES))

# 4. rotation fires, and lands near the target
if len(remuxed) < 8:
    fails.append('only %d clips from 300s at a 30s target — rotation is not firing' % len(remuxed))
for path, frames, span in remuxed[:-1]:
    if not 30.0 <= span <= 33.5:
        fails.append('%s span %.2fs is not near the 30s target' % (os.path.basename(path), span))

# 5. budget prune drops oldest first and never the open segment
S.cfg['clips_max_gb'] = 0.5
for i in range(6):
    p = os.path.join(TD, 'clips', 'clip_20200101_00000%d.mp4' % i)
    f = open(p, 'wb'); f.truncate(120 * MB); f.close()
    os.utime(p, (1000 + i, 1000 + i))
S._clip_enforce_budget()
left = sorted(f for f in os.listdir(TD + '/clips') if f.startswith('clip_20200101'))
print('after prune (0.5 GB budget, 6 x 120 MB): %s' % left)
if not 1 <= len(left) <= 5:
    fails.append('prune kept %d files' % len(left))
if left and left[0] != 'clip_20200101_000002.mp4':
    fails.append('prune did not delete oldest first: kept %s' % left)

# 6. _clip_rows(): duration comes from the index where it exists, and is
#    inferred from the next clip's start where it does not.
for f in os.listdir(TD + '/clips'):
    os.remove(os.path.join(TD, 'clips', f))
S.cfg['clips_secs'] = 600
base = 1789000000
names = []
for i in range(4):
    t = base + i * 600
    n = 'clip_' + S.time.strftime('%Y%m%d_%H%M%S', S.time.localtime(t)) + '.mp4'
    names.append(n)
    fh = open(os.path.join(TD, 'clips', n), 'wb'); fh.truncate(1024); fh.close()
# index knows only the second clip, and says it was short (a restart cut it)
S._clip_index_add(names[1], base + 600, 412.5, 4000, 9.7)
rows = S._clip_rows()
got = {r['name']: round(r['duration'], 1) for r in rows}
print('durations:', [got[n] for n in names])
if got[names[1]] != 412.5:
    fails.append('indexed clip should report its measured 412.5s, got %s' % got[names[1]])
for n in (names[0], names[2]):
    if got[n] != 600:
        fails.append('%s should infer 600s from the next start, got %s' % (n, got[n]))
if got[names[3]] != 600:
    fails.append('newest clip should fall back to clips_secs, got %s' % got[names[3]])
if [r['name'] for r in rows] != names:
    fails.append('rows must be ascending by start time')
if any(abs(r['end'] - (r['started'] + r['duration'])) > 0.01 for r in rows):
    fails.append('end must equal started + duration')

shutil.rmtree(TD, ignore_errors=True)
print()
if fails:
    print('FAIL:')
    for f in fails:
        print('  - ' + f)
    sys.exit(1)
print('ALL CHECKS PASSED')
