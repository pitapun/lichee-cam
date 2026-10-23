"""Retention checks for the events dir: duplicate-h264 reclaim and byte budget.

Run from the repo root:  python3 tests/test_event_retention.py

The ordering check is the one that matters — reclaiming a duplicate event.h264
rewrites the parent dir's mtime, so anything that prunes by mtime deletes the
wrong events.  _event_dir_epoch() reads the save time out of the dir name instead.
"""
import os, sys, shutil, tempfile
TD = tempfile.mkdtemp(prefix='evtest-')
os.makedirs(TD + '/events')
os.environ['NINTI_CONFIG']    = TD + '/cfg.json'
os.environ['NINTI_EVENT_DIR'] = TD + '/events'
os.environ['NINTI_CLIP_DIR']  = TD + '/clips'
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import sidecar as S

ED = TD + '/events'
MB = 1024 * 1024

def mk(name, mtime, mp4_mb, h264_mb):
    d = os.path.join(ED, name)
    os.makedirs(d, exist_ok=True)
    # sparse: getsize() reports the full length, so the budget maths is real
    # without writing gigabytes to /tmp
    if mp4_mb:
        f = open(d + '/event.mp4', 'wb'); f.truncate(int(mp4_mb * MB)); f.close()
    if h264_mb:
        f = open(d + '/event.h264', 'wb'); f.truncate(int(h264_mb * MB)); f.close()
    os.utime(d, (mtime, mtime))
    return d

# 10 events, oldest first; two of them never produced an mp4
for i in range(10):
    mk('event_%05d_save_%d' % (i, 1700000000 + i), 1700000000 + i,
       mp4_mb=0 if i in (3, 7) else 100, h264_mb=120)

fails = []
S.cfg['events_max_gb'] = 0.5          # 512 MB — the floor _events_max_bytes() clamps to

S._cleanup_event_storage()

# 1. duplicate h264 gone where an mp4 exists; kept where it does not
for i in range(10):
    d = os.path.join(ED, 'event_%05d_save_%d' % (i, 1700000000 + i))
    if not os.path.isdir(d):
        continue
    raw = os.path.exists(d + '/event.h264')
    mp4 = os.path.exists(d + '/event.mp4')
    if mp4 and raw:
        fails.append('event %d: mp4 exists but h264 was not reclaimed' % i)
    if not mp4 and not raw:
        fails.append('event %d: h264 deleted with no mp4 to replace it' % i)

# 2. total under budget, oldest deleted first
left = sorted(os.listdir(ED))
total = sum(S._dir_bytes(os.path.join(ED, n)) for n in left)
print('kept %d/%d events, %.0f MB (budget 512 MB)' % (len(left), 10, total / MB))
print('kept:', left)
if total > S._events_max_bytes():
    fails.append('total %.0f MB over the %.0f MB budget' % (total / MB, S._events_max_bytes() / MB))
kept_idx = [int(n.split('_')[1]) for n in left]
gone_idx = [i for i in range(10) if i not in kept_idx]
if gone_idx and kept_idx and max(gone_idx) > min(kept_idx):
    fails.append('prune deleted event %d while keeping older event %d'
                 % (max(gone_idx), min(kept_idx)))
print('deleted:', gone_idx, '-> all older than every kept event:',
      not (gone_idx and kept_idx and max(gone_idx) > min(kept_idx)))

# 3. a dir with no mp4 is still prunable by age (not pinned forever)
if len(left) == 10:
    fails.append('nothing was pruned at a 512 MB budget')

shutil.rmtree(TD, ignore_errors=True)
print()
if fails:
    print('FAIL:'); [print('  - ' + f) for f in fails]; sys.exit(1)
print('ALL CHECKS PASSED')
