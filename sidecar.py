#!/usr/bin/env python3
"""
NintiDetect sidecar
  - Manages stream_yolo subprocess (start / restart on threshold change)
  - Manages RTSP stack (S99gc4653rtsp start/stop), mutually exclusive with NintiDetect
  - Listens on UDP 5005 for detections, applies zone filter
  - Pushes filtered detections to browser via Server-Sent Events
  - Serves web UI on port 7778  (MJPEG still direct from :7777)
"""

import json, os, subprocess, threading, time, socket, sys, io
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse
from queue import Queue, Empty
import http.client

# ---- Paths / ports ----
CONFIG_FILE = os.environ.get('NINTI_CONFIG', '/root/ninti_config.json')
STREAM_BIN  = os.environ.get('STREAM_BIN',  '/root/stream_yolo')
MODEL       = os.environ.get('MODEL',        '/root/yolov5_cv181x.cvimodel')
RTSP_INITD  = '/etc/init.d/S99gc4653rtsp'
LD_PATH     = '/mnt/system/usr/lib:/usr/bin/lib:/root/libs_patch'
UDP_PORT    = 5005
HTTP_PORT   = 7778
MJPEG_PORT  = 7777
EVENT_DIR   = os.environ.get('NINTI_EVENT_DIR', '/root/ninti_events')
EVENT_FILE  = os.path.join(EVENT_DIR, 'events.jsonl')

DEFAULT_CONFIG = {
    'mode': 'ninti',
    'threshold': 0.50,
    'zones': [],
    'filter_classes': ['person', 'vehicle', 'animal'],
    'motion_enabled': True,
    'motion_sensitivity': 20,
}

# ---- State ----
config_lock  = threading.Lock()
sse_clients  = []          # list of Queue
yolo_proc    = None
yolo_lock    = threading.Lock()
mode_lock    = threading.Lock()
tracks_lock  = threading.Lock()
tracks       = {}
next_track_id = 1

TRACK_IOU_THRESHOLD = 0.20
TRACK_CENTER_THRESHOLD = 0.25
TRACK_CONFIRM_HITS = 2
TRACK_LOST_TIMEOUT = 5.0    # object not detected for 5s -> gone
TRACK_STILL_TIMEOUT = 5.0   # object not moving for 5s -> gone
TRACK_MOVE_THRESHOLD = 0.04 # min center shift (normalised) to count as movement

cfg = DEFAULT_CONFIG.copy()
if os.path.exists(CONFIG_FILE):
    try:
        with open(CONFIG_FILE) as f:
            cfg.update(json.load(f))
    except Exception as e:
        print(f'[config] load error: {e}')

def save_cfg():
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(cfg, f, indent=2)
    except Exception as e:
        print(f'[config] save error: {e}')

# ---- Point-in-polygon (ray casting) ----
def pip(x, y, poly):
    n, inside, j = len(poly), False, len(poly) - 1
    for i in range(n):
        xi, yi = poly[i]; xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside

def in_any_zone(cx, cy, zones):
    active = [z for z in zones if z.get('enabled', True) and len(z.get('points', [])) >= 3]
    if not active:
        return True
    return any(pip(cx, cy, z['points']) for z in active)

# ---- Simple tracking / event recorder ----
def _bbox(det):
    return (float(det.get('x1', 0)), float(det.get('y1', 0)),
            float(det.get('x2', 0)), float(det.get('y2', 0)))

def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    aa = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    ba = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    return inter / (aa + ba - inter + 1e-9)

def _center_dist(a, b):
    return ((float(a.get('cx', 0.5)) - float(b.get('cx', 0.5))) ** 2 +
            (float(a.get('cy', 0.5)) - float(b.get('cy', 0.5))) ** 2) ** 0.5

def _write_event(kind, tr, now):
    try:
        os.makedirs(EVENT_DIR, exist_ok=True)
        rec = {
            'type': kind,
            'track_id': tr['id'],
            'cat': tr.get('cat'),
            'cls': tr.get('cls'),
            'name': tr.get('name'),
            'first_seen': tr.get('first_seen'),
            'last_seen': tr.get('last_seen'),
            'duration_s': round(tr.get('last_seen', now) - tr.get('first_seen', now), 3),
            'hits': tr.get('hits', 0),
            'best_score': tr.get('best_score', 0),
            'bbox': tr.get('bbox'),
            't': now,
        }
        with open(EVENT_FILE, 'a') as f:
            f.write(json.dumps(rec, separators=(',', ':')) + '\n')
    except Exception as e:
        print(f'[events] write error: {e}')

def _expire_tracks(now=None):
    if now is None:
        now = time.time()
    expired = []
    with tracks_lock:
        for tid, tr in list(tracks.items()):
            lost  = now - tr.get('last_seen',  now) > TRACK_LOST_TIMEOUT
            still = now - tr.get('last_moved', now) > TRACK_STILL_TIMEOUT
            if lost or still:
                if tr.get('confirmed'):
                    _write_event('end', tr, now)
                expired.append(tid)
        for tid in expired:
            tr = tracks.pop(tid, {})
            if tr.get('hits', 0) >= 1:
                # any tracked object gets saved
                if not tr.get('confirmed'):
                    _write_event('end', tr, now)  # write end event if start wasn't written
                msg = 'data: ' + json.dumps({'_save': True, 'track_id': tid,
                                              'name': tr.get('name', '?'),
                                              'hits': tr.get('hits', 0),
                                              'duration_s': round(now - tr.get('first_seen', now), 1)}) + '\n\n'
            else:
                msg = 'data: ' + json.dumps({'_discard': True, 'track_id': tid,
                                              'name': tr.get('name', '?'),
                                              'hits': tr.get('hits', 0)}) + '\n\n'
            for q in list(sse_clients):
                try: q.put_nowait(msg)
                except: pass

def _update_track(det):
    global next_track_id
    now = time.time()
    _expire_tracks(now)
    det_box = _bbox(det)
    best_id, best_score = None, -999.0
    with tracks_lock:
        for tid, tr in tracks.items():
            if tr.get('cls') != det.get('cls'):
                continue
            iou = _iou(det_box, tr.get('bbox', det_box))
            dist = _center_dist(det, tr.get('last_det', det))
            if iou < TRACK_IOU_THRESHOLD and dist > TRACK_CENTER_THRESHOLD:
                continue
            score = iou - dist
            if score > best_score:
                best_id, best_score = tid, score

        if best_id is None:
            best_id = next_track_id
            next_track_id += 1
            tracks[best_id] = {
                'id': best_id,
                'cat': det.get('cat'),
                'cls': det.get('cls'),
                'name': det.get('name'),
                'first_seen': now,
                'last_seen': now,
                'last_moved': now,
                'hits': 0,
                'confirmed': False,
                'best_score': 0.0,
                'bbox': det_box,
                'last_det': det.copy(),
            }

        tr = tracks[best_id]
        # check if object moved significantly
        prev = tr.get('last_det', det)
        dx = float(det.get('cx', 0.5)) - float(prev.get('cx', 0.5))
        dy = float(det.get('cy', 0.5)) - float(prev.get('cy', 0.5))
        if (dx*dx + dy*dy) ** 0.5 >= TRACK_MOVE_THRESHOLD:
            tr['last_moved'] = now
        tr['last_seen'] = now
        tr['hits'] += 1
        tr['bbox'] = det_box
        tr['last_det'] = det.copy()
        tr['best_score'] = max(float(tr.get('best_score', 0.0)), float(det.get('score', 0.0)))
        if not tr.get('confirmed') and tr['hits'] >= TRACK_CONFIRM_HITS:
            tr['confirmed'] = True
            _write_event('start', tr, now)

        det['track_id'] = best_id
        det['track_hits'] = tr['hits']
        det['track_confirmed'] = tr['confirmed']
    return det

# ---- RTSP lifecycle ----
def _rtsp_running():
    try:
        out = subprocess.check_output(['ps'], stderr=subprocess.DEVNULL).decode(errors='replace')
        return 'mediamtx' in out
    except Exception:
        return False

def _start_rtsp():
    if os.path.exists(RTSP_INITD) and os.access(RTSP_INITD, os.X_OK):
        subprocess.call([RTSP_INITD, 'start'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print('[rtsp] started')
    else:
        print(f'[rtsp] init script not executable: {RTSP_INITD}')

def _stop_rtsp():
    if os.path.exists(RTSP_INITD) and os.access(RTSP_INITD, os.X_OK):
        subprocess.call([RTSP_INITD, 'stop'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # also kill sample_venc and ffmpeg which hold VPSS group 0
    for proc in ('sample_venc', 'ffmpeg', 'mediamtx'):
        subprocess.call(['killall', proc], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)
    print('[rtsp] stopped')

# ---- stream_yolo lifecycle ----
def _yolo_running():
    with yolo_lock:
        return yolo_proc is not None and yolo_proc.poll() is None

def _start_yolo_inner():
    global yolo_proc
    with yolo_lock:
        if yolo_proc and yolo_proc.poll() is None:
            yolo_proc.terminate()
            try: yolo_proc.wait(3)
            except: yolo_proc.kill()
        env = os.environ.copy()
        env['LD_LIBRARY_PATH'] = LD_PATH
        thresh = str(cfg.get('threshold', 0.45))
        print(f'[yolo] starting with threshold={thresh}')
        yolo_log = open('/tmp/stream_yolo.log', 'w')
        yolo_proc = subprocess.Popen(
            [STREAM_BIN, MODEL, '80', '640', thresh, str(UDP_PORT), '1280', '720'],
            env=env, stdout=yolo_log, stderr=yolo_log)

def start_yolo():
    threading.Thread(target=_start_yolo_inner, daemon=True).start()

def stop_yolo():
    global yolo_proc
    with yolo_lock:
        if yolo_proc:
            yolo_proc.terminate()
            try: yolo_proc.wait(8)
            except: yolo_proc.kill()
            yolo_proc = None
    print('[yolo] stopped')

# ---- D-state watchdog ----
_dstate_count = 0

def _proc_state(pid):
    try:
        with open(f'/proc/{pid}/status') as f:
            for line in f:
                if line.startswith('State:'):
                    return line.split()[1]
    except Exception:
        return None

def _watchdog():
    global _dstate_count
    import subprocess as _sp
    while True:
        time.sleep(10)
        with yolo_lock:
            pid = yolo_proc.pid if yolo_proc else None
        if pid is None:
            _dstate_count = 0
            continue
        st = _proc_state(pid)
        if st == 'D':
            _dstate_count += 1
            print(f'[watchdog] stream_yolo pid {pid} D-state ({_dstate_count}/3)')
            if _dstate_count >= 6:
                print('[watchdog] D-state persistent, rebooting')
                _sp.call(['reboot', '-f'])
        else:
            _dstate_count = 0

# ---- Motion detector (frame difference, independent of AI) ----
MOTION_SENSITIVITY = 20   # pixel diff threshold per channel (0-255)
MOTION_MIN_PIXELS  = 500  # min changed pixels to count as motion
MOTION_COOLDOWN    = 0.3  # seconds between motion SSE events

MOTION_CHECK_INTERVAL = 0.20  # only process one frame per 200ms; keep MJPEG queue free

def _motion_detector():
    from PIL import Image
    import http.client as _hc
    prev_gray = None
    last_fire  = 0.0
    last_check = 0.0
    while True:
        try:
            conn = _hc.HTTPConnection('127.0.0.1', MJPEG_PORT, timeout=5)
            conn.request('GET', '/')
            resp = conn.getresponse()
            buf = b''
            while True:
                chunk = resp.read(4096)
                if not chunk:
                    break
                buf += chunk
                # Find complete JPEG frames (FF D8 ... FF D9)
                while True:
                    s = buf.find(b'\xff\xd8')
                    if s < 0:
                        buf = b''
                        break
                    e = buf.find(b'\xff\xd9', s + 2)
                    if e < 0:
                        buf = buf[s:]
                        break
                    jpg = buf[s:e + 2]
                    buf = buf[e + 2:]
                    try:
                        with config_lock:
                            motion_on  = cfg.get('motion_enabled', True)
                            sensitivity = cfg.get('motion_sensitivity', MOTION_SENSITIVITY)
                        if not motion_on:
                            prev_gray = None
                            continue
                        # Rate-limit: skip frames to avoid blocking the MJPEG write queue
                        now_t = time.time()
                        if now_t - last_check < MOTION_CHECK_INTERVAL:
                            continue
                        last_check = now_t
                        img = Image.open(io.BytesIO(jpg)).convert('L').resize((80, 45))
                        pixels = list(img.getdata())
                        if prev_gray is not None:
                            changed = sum(1 for a, b in zip(pixels, prev_gray)
                                          if abs(a - b) > sensitivity)
                            if changed >= MOTION_MIN_PIXELS:
                                now = time.time()
                                if now - last_fire > MOTION_COOLDOWN:
                                    last_fire = now
                                    msg = 'data: ' + json.dumps(
                                        {'_motion': True, 'pixels': changed}) + '\n\n'
                                    for q in list(sse_clients):
                                        try: q.put_nowait(msg)
                                        except: pass
                        prev_gray = pixels
                    except Exception:
                        pass
        except Exception:
            prev_gray = None
            time.sleep(2)

# ---- Mode switch ----
def set_mode(new_mode):
    with mode_lock:
        print(f'[mode] switching to {new_mode}')
        if new_mode == 'ninti':
            _stop_rtsp()
            time.sleep(0.5)
            threading.Thread(target=_start_yolo_inner, daemon=True).start()
        elif new_mode == 'rtsp':
            stop_yolo()
            time.sleep(0.5)
            _start_rtsp()
        with config_lock:
            cfg['mode'] = new_mode
        save_cfg()
        print(f'[mode] now in {new_mode} mode')

# ---- UDP listener ----
def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('127.0.0.1', UDP_PORT))
    sock.settimeout(0.5)
    print(f'[udp] listening on 127.0.0.1:{UDP_PORT}')
    while True:
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            _expire_tracks()
            continue
        try:
            det = json.loads(data)
            if '_fps' in det:
                msg = 'data: ' + json.dumps(det) + '\n\n'
                for q in list(sse_clients):
                    try: q.put_nowait(msg)
                    except: pass
                continue
            with config_lock:
                zones = cfg.get('zones', [])
                filt  = cfg.get('filter_classes', [])
            if filt and det.get('cat') not in filt:
                continue
            if not in_any_zone(det.get('cx', 0.5), det.get('cy', 0.5), zones):
                continue
            det = _update_track(det)
            msg = 'data: ' + json.dumps(det) + '\n\n'
            dead = []
            for q in sse_clients:
                try: q.put_nowait(msg)
                except: dead.append(q)
            for q in dead:
                if q in sse_clients: sse_clients.remove(q)
        except Exception:
            pass

# ---- Embedded HTML (single-file) ----
HTML = b'''<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>NintiDetect</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#111;color:#eee;display:flex;height:100vh;overflow:hidden}
#state{width:160px;padding:12px 10px;background:#141414;display:flex;flex-direction:column;gap:0;font-size:12px;border-right:1px solid #2a2a2a;overflow-y:auto}
#state h2{font-size:10px;color:#555;border-bottom:1px solid #222;padding-bottom:3px;margin-bottom:10px;letter-spacing:.05em}
.pipeline{display:flex;flex-direction:column;align-items:stretch;gap:0}
.pnode{border:1px solid #2a2a2a;border-radius:5px;padding:7px 10px;background:#1a1a1a;transition:background .15s,border-color .15s,color .15s;cursor:default;position:relative}
.pnode .pname{font-size:11px;font-weight:bold;color:#555;letter-spacing:.03em;transition:color .15s}
.pnode .pinfo{font-size:10px;color:#444;margin-top:2px;min-height:12px;transition:color .15s}
.pnode.active{background:#0d2a0d;border-color:#2a6a2a}
.pnode.active .pname{color:#4f4}
.pnode.active .pinfo{color:#2a2}
.pnode.flash-save{background:#0d2a1a;border-color:#2a6a4a}
.pnode.flash-save .pname{color:#4fa}
.pnode.flash-discard{background:#2a1a0d;border-color:#6a4a2a}
.pnode.flash-discard .pname{color:#fa8}
.parrow{text-align:center;color:#333;font-size:14px;line-height:1;margin:2px 0;user-select:none}
.parrow.active{color:#2a6a2a}
.pardiv{display:flex;justify-content:space-around;color:#333;font-size:11px;line-height:1;margin:2px 0}
.pstat{margin-top:14px;border-top:1px solid #222;padding-top:8px;display:flex;flex-direction:column;gap:5px}
.st-row{display:flex;flex-direction:column;gap:1px}
.st-label{font-size:10px;color:#555}
.st-val{font-size:12px;color:#ccc}
.st-val.on{color:#0f0}
.st-val.off{color:#555}
.st-val.warn{color:#fa5}
#left{flex:1;position:relative;display:flex;align-items:center;justify-content:center;background:#000}
#stream{display:block;max-width:100%;max-height:100%;image-rendering:pixelated}
#overlay{position:absolute;cursor:crosshair}
#right{width:300px;padding:10px;overflow-y:auto;background:#1a1a1a;display:flex;flex-direction:column;gap:8px;font-size:12px}
h2{font-size:11px;color:#888;border-bottom:1px solid #333;padding-bottom:3px;margin-bottom:2px}
input[type=range]{width:100%}
input[type=text]{background:#222;color:#eee;border:1px solid #555;padding:3px 5px;font-family:monospace;font-size:12px}
button{background:#2a2a2a;color:#ddd;border:1px solid #555;padding:4px 9px;cursor:pointer;font-family:monospace;font-size:12px}
button:hover{background:#3a3a3a}
button.red{border-color:#633;color:#f88}
button.act{background:#1a3a1a;border-color:#4a8a4a;color:#8f8}
.row{display:flex;gap:6px;align-items:center;flex-wrap:wrap}
#log,#events{height:120px;overflow-y:auto;background:#0a0a0a;padding:5px;border:1px solid #2a2a2a}
.ll{color:#0f0;padding:1px 0}
.ev{color:#8cf;padding:1px 0}
#status{font-size:11px;color:#666;min-height:16px}
.zi{background:#1e1e1e;border:1px solid #3a3a3a;padding:5px;border-radius:3px;margin-bottom:4px}
.zn{font-weight:bold}
.hint{color:#666;font-size:11px}
.mode-radio{display:flex;gap:10px}
.mode-radio label{display:flex;align-items:center;gap:4px;cursor:pointer;padding:4px 8px;border:1px solid #444;border-radius:3px}
.mode-radio label.active{border-color:#4a8a4a;background:#1a3a1a;color:#8f8}
.dot{width:7px;height:7px;border-radius:50%;background:#555;display:inline-block}
.dot.on{background:#0f0}
.dot.off{background:#555}
</style>
</head>
<body>
<div id="state">
  <h2>PIPELINE</h2>
  <div class="pipeline">
    <div class="pnode" id="pn-motion">
      <div class="pname">MOTION DETECT</div>
      <div class="pinfo" id="pi-motion">-</div>
    </div>
    <div class="parrow" id="pa-motion">&#x25BC;</div>
    <div class="pnode" id="pn-ai">
      <div class="pname">AI DETECT</div>
      <div class="pinfo" id="pi-ai">-</div>
    </div>
    <div class="parrow" id="pa-ai">&#x25BC;</div>
    <div class="pnode" id="pn-track">
      <div class="pname">AI TRACKING</div>
      <div class="pinfo" id="pi-track">-</div>
    </div>
    <div class="pardiv" id="pa-track">
      <span>&#x25C4; discard</span>
      <span>save &#x25BA;</span>
    </div>
    <div style="display:flex;gap:6px">
      <div class="pnode" id="pn-discard" style="flex:1">
        <div class="pname">DISCARD</div>
        <div class="pinfo" id="pi-discard"></div>
      </div>
      <div class="pnode" id="pn-save" style="flex:1">
        <div class="pname">SAVE</div>
        <div class="pinfo" id="pi-save"></div>
      </div>
    </div>
  </div>
  <div class="pstat">
    <div class="st-row"><span class="st-label">mode</span><span class="st-val" id="stMode">-</span></div>
    <div class="st-row"><span class="st-label">detector</span><span class="st-val off" id="stYolo">-</span></div>
    <div class="st-row"><span class="st-label">tracks</span><span class="st-val" id="stTracks">0</span></div>
    <div class="st-row"><span class="st-label">threshold</span><span class="st-val" id="stThresh">-</span></div>
    <div class="st-row"><span class="st-label">fps</span><span class="st-val" id="stFps">-</span></div>
    <div class="st-row"><span class="st-label">last det</span><span class="st-val" id="stLastDet" style="font-size:10px;line-height:1.4">-</span></div>
  </div>
</div>
<div id="left">
  <img id="stream" src="" alt="stream offline">
  <canvas id="overlay"></canvas>
</div>
<div id="right">

<div>
<h2>CONNECTION</h2>
<div class="row">
  <input id="ip" type="text" placeholder="board IP" style="flex:1" value="">
  <button onclick="connect()">Connect</button>
</div>
<div id="status"></div>
</div>

<div>
<h2>MODE</h2>
<div class="mode-radio" id="modeRadio">
  <label id="lbl_ninti"><input type="radio" name="mode" value="ninti" onchange="pendingMode='ninti';updateModeLabels()"> NintiDetect</label>
  <label id="lbl_rtsp"><input type="radio" name="mode" value="rtsp" onchange="pendingMode='rtsp';updateModeLabels()"> RTSP</label>
</div>
<div class="row" style="margin-top:4px">
  <button onclick="applyMode()" style="flex:1">Switch Mode</button>
  <span class="hint" id="modeStatus"></span>
</div>
<div class="row hint" style="margin-top:2px">
  <span class="dot" id="dotYolo"></span> detector
  &nbsp;&nbsp;
  <span class="dot" id="dotRtsp"></span> rtsp
</div>
</div>

<div>
<h2>THRESHOLD</h2>
<div class="row">
  <input type="range" id="th" min="0.05" max="0.95" step="0.05" value="0.45"
         oninput="document.getElementById('thv').textContent=parseFloat(this.value).toFixed(2)">
  <span id="thv" style="width:32px">0.45</span>
</div>
<button onclick="applyThresh()" style="margin-top:4px;width:100%">Apply &amp; restart detector</button>
</div>

<div>
<h2>MOTION DETECT</h2>
<div class="row" style="margin-bottom:4px">
  <button id="btnMotion" onclick="toggleMotion()" style="flex:1">-</button>
</div>
<div class="row">
  <span class="hint" style="width:60px">sensitivity</span>
  <input type="range" id="mth" min="5" max="50" step="5" value="20" style="flex:1"
         oninput="document.getElementById('mthv').textContent=this.value">
  <span id="mthv" style="width:28px">20</span>
</div>
<button onclick="applyMotion()" style="margin-top:4px;width:100%">Apply</button>
</div>

<div>
<h2>ZONES <span class="hint">(click stream to draw)</span></h2>
<div class="row" style="margin-bottom:4px">
  <input id="zname" type="text" placeholder="zone name" style="flex:1">
  <button onclick="newZone()">+ New</button>
  <button onclick="cancelZone()" class="red">Cancel</button>
</div>
<div id="zlist"></div>
</div>

<div>
<h2>FILTER CLASSES</h2>
<div id="fclasses"></div>
</div>

<button onclick="saveConf()" style="width:100%;padding:6px">Save Config</button>

<div>
<h2>DETECTIONS</h2>
<div id="log"></div>
</div>

<div>
<h2>RECENT EVENTS</h2>
<div class="row" style="margin-bottom:4px">
  <button onclick="loadEvents()" style="flex:1">Refresh</button>
  <span class="hint" id="eventStatus"></span>
</div>
<div id="events"></div>
</div>

</div>

<script>
const PALETTE=['#f55','#5af','#5f5','#fa5','#a5f','#ff5','#5ff'];
let cfg={mode:'ninti',threshold:0.45,zones:[],filter_classes:['person','vehicle','animal']};
let drawing=null;
let hover=null;
let flashes=[];
let ip='';
let es=null;
let pendingMode='ninti';

const streamEl=document.getElementById('stream');
const cvs=document.getElementById('overlay');
const ctx=cvs.getContext('2d');

if(location.hostname && location.hostname!=='localhost'){
  document.getElementById('ip').value=location.hostname;
}

function getip(){return document.getElementById('ip').value.trim()||location.hostname;}

function connect(){
  ip=getip();
  streamEl.src='http://'+ip+':7778/stream';
  loadConf();
  loadEvents();
  openSSE();
  pollStatus();
  setInterval(loadEvents, 5000);
}

function loadConf(){
  fetch('http://'+ip+':7778/api/config')
    .then(r=>r.json()).then(c=>{
      cfg=c;
      pendingMode=c.mode||'ninti';
      document.getElementById('th').value=c.threshold;
      document.getElementById('thv').textContent=parseFloat(c.threshold).toFixed(2);
      document.querySelectorAll('input[name=mode]').forEach(r=>{r.checked=(r.value===pendingMode);});
      updateModeLabels();
      const ms=c.motion_sensitivity||20;
      document.getElementById('mth').value=ms;
      document.getElementById('mthv').textContent=ms;
      updateMotionBtn();
      renderZones(); renderClasses();
    }).catch(e=>setStatus('load config failed: '+e));
}

function saveConf(){
  fetch('http://'+ip+':7778/api/config',{
    method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(cfg)
  }).then(r=>r.json()).then(()=>setStatus('Saved.')).catch(e=>setStatus('save failed: '+e));
}

function loadEvents(){
  const el=document.getElementById('events');
  const st=document.getElementById('eventStatus');
  if(!ip)return;
  fetch('http://'+ip+':7778/api/events')
    .then(r=>r.json()).then(rows=>{
      el.innerHTML='';
      st.textContent=rows.length+' rows';
      if(!rows.length){el.innerHTML='<div class="hint">No events yet</div>';return;}
      rows.slice().reverse().forEach(e=>{
        const div=document.createElement('div');
        div.className='ev';
        const dur=e.duration_s!==undefined ? ` ${Number(e.duration_s).toFixed(1)}s` : '';
        const sc=e.best_score!==undefined ? ` ${(Number(e.best_score)*100).toFixed(0)}%` : '';
        div.textContent=`${e.type} #${e.track_id} ${e.cat||''}/${e.name||''}${sc}${dur}`;
        el.appendChild(div);
      });
      el.scrollTop=el.scrollHeight;
    }).catch(e=>{
      st.textContent='failed';
    });
}

function toggleMotion(){
  cfg.motion_enabled=!cfg.motion_enabled;
  updateMotionBtn();
  fetch('http://'+ip+':7778/api/config',{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({motion_enabled:cfg.motion_enabled})
  }).catch(e=>setStatus(''+e));
}
function applyMotion(){
  cfg.motion_sensitivity=parseInt(document.getElementById('mth').value);
  fetch('http://'+ip+':7778/api/config',{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({motion_sensitivity:cfg.motion_sensitivity})
  }).then(()=>setStatus('Motion settings saved.')).catch(e=>setStatus(''+e));
}
function updateMotionBtn(){
  const btn=document.getElementById('btnMotion');
  if(!btn)return;
  btn.textContent=cfg.motion_enabled?'Enabled (click to disable)':'Disabled (click to enable)';
  btn.className=cfg.motion_enabled?'act':'red';
}
function applyThresh(){
  cfg.threshold=parseFloat(document.getElementById('th').value);
  fetch('http://'+ip+':7778/api/config',{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({...cfg,_restart:true})
  }).then(r=>r.json()).then(()=>setStatus('Detector restarting...')).catch(e=>setStatus(''+e));
}

function applyMode(){
  const m=pendingMode;
  document.getElementById('modeStatus').textContent='switching...';
  fetch('http://'+ip+':7778/api/mode',{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode:m})
  }).then(r=>r.json()).then(d=>{
    cfg.mode=d.mode;
    document.getElementById('modeStatus').textContent='done';
    if(d.mode==='ninti'){
      streamEl.src='http://'+ip+':7778/stream';
    } else {
      streamEl.src='';
      streamEl.alt='RTSP mode - use rtsp://'+ip+':8554/live';
    }
    setTimeout(()=>{document.getElementById('modeStatus').textContent='';},3000);
  }).catch(e=>{document.getElementById('modeStatus').textContent='failed';setStatus(''+e);});
}

function updateModeLabels(){
  document.getElementById('lbl_ninti').className=pendingMode==='ninti'?'active':'';
  document.getElementById('lbl_rtsp').className=pendingMode==='rtsp'?'active':'';
}

function pollStatus(){
  function update(){
    fetch('http://'+ip+':7778/api/status')
      .then(r=>r.json()).then(s=>{
        document.getElementById('dotYolo').className='dot '+(s.yolo?'on':'off');
        document.getElementById('dotRtsp').className='dot '+(s.rtsp?'on':'off');
        // left state panel
        const stMode=document.getElementById('stMode');
        stMode.textContent=s.mode||'-';
        stMode.className='st-val '+(s.mode==='ninti'?'on':s.mode==='rtsp'?'warn':'');
        const stY=document.getElementById('stYolo');
        stY.textContent=s.yolo?'running':'stopped';
        stY.className='st-val '+(s.yolo?'on':'off');
        const stR=document.getElementById('stRtsp');
        stR.textContent=s.rtsp?'running':'stopped';
        stR.className='st-val '+(s.rtsp?'on':'off');
        const trackCount=s.tracks||0;
        document.getElementById('stTracks').textContent=trackCount;
        document.getElementById('stThresh').textContent=parseFloat(cfg.threshold||0).toFixed(2);
        // keep TRACKING node lit while objects are being tracked
        const tn=document.getElementById('pn-track');
        if(tn){
          if(trackCount>0){
            if(!tn.classList.contains('active')) pnSet('track','active',trackCount+' object'+(trackCount>1?'s':''));
            else document.getElementById('pi-track').textContent=trackCount+' object'+(trackCount>1?'s':'');
          } else {
            pnClear('track');
          }
        }
      }).catch(()=>{});
  }
  update();
  setInterval(update, 3000);
}

// SSE
function openSSE(){
  if(es)es.close();
  es=new EventSource('http://'+ip+':7778/events');
  es.onmessage=e=>{try{const d=JSON.parse(e.data);if(d._fps!==undefined)onFps(d);else if(d._motion)onMotion(d);else if(d._save)onSave(d);else if(d._discard)onDiscard(d);else onDet(d);}catch{}};
  es.onopen=()=>setStatus('Connected');
  es.onerror=()=>setStatus('SSE offline, retrying...');
}

// Pipeline node helpers
const PIPELINE_NODES=['motion','ai','track','save','discard'];
const pnTimers={};
function pnSet(id,cls,info,dur){
  const el=document.getElementById('pn-'+id);
  const pi=document.getElementById('pi-'+id);
  if(!el)return;
  el.className='pnode '+(cls||'active');
  if(pi&&info!==undefined)pi.textContent=info;
  const arr=document.getElementById('pa-'+id);
  if(arr)arr.className='parrow active';
  clearTimeout(pnTimers[id]);
  if(dur){
    pnTimers[id]=setTimeout(()=>{
      el.className='pnode';
      if(arr)arr.className='parrow';
      if(pi)pi.textContent='';
    },dur);
  }
}
function pnClear(id){
  const el=document.getElementById('pn-'+id);
  const pi=document.getElementById('pi-'+id);
  const arr=document.getElementById('pa-'+id);
  if(el)el.className='pnode';
  if(pi)pi.textContent='';
  if(arr)arr.className='parrow';
}

function onFps(d){
  const el=document.getElementById('stFps');
  if(el) el.textContent=d._fps.toFixed(1)+' fps';
}
function onDet(d){
  const log=document.getElementById('log');
  const ln=document.createElement('div');
  ln.className='ll';
  ln.textContent=`${d.cat} ${d.name} ${(d.score*100).toFixed(0)}% (${d.cx.toFixed(2)},${d.cy.toFixed(2)})`;
  log.appendChild(ln);
  if(log.children.length>80)log.removeChild(log.firstChild);
  log.scrollTop=log.scrollHeight;
  flashes.push({...d,t:Date.now()});
  const now=new Date();
  const ts=now.getHours().toString().padStart(2,'0')+':'+
    now.getMinutes().toString().padStart(2,'0')+':'+
    now.getSeconds().toString().padStart(2,'0');
  document.getElementById('stLastDet').textContent=ts+' '+d.name;

  // Drive pipeline states (motion handled separately)
  const score=(d.score*100).toFixed(0)+'%';
  const label=d.name+' '+score;
  pnClear('motion');
  pnSet('ai','active',label,500);
  const hits=d.track_hits||1;
  setTimeout(()=>pnSet('track','active','#'+d.track_id+' hits '+hits, 6000),160);
}

function onMotion(d){
  const aiActive=document.getElementById('pn-ai').classList.contains('active');
  const trActive=document.getElementById('pn-track').classList.contains('active');
  if(aiActive||trActive) return;
  pnSet('motion','active',d.pixels+' px',350);
}

function resetToMotion(){
  pnClear('ai'); pnClear('track'); pnClear('save'); pnClear('discard');
  pnSet('motion','active','watching',0);
}

function onSave(d){
  pnClear('track');
  pnClear('ai');
  pnSet('save','flash-save','#'+d.track_id+' '+d.duration_s+'s',1200);
  setTimeout(resetToMotion, 1400);
}

function onDiscard(d){
  pnClear('track');
  pnClear('ai');
  pnSet('discard','flash-discard','#'+d.track_id+' ('+d.hits+'hits)',800);
  setTimeout(resetToMotion, 1000);
}

// Canvas
function syncCvs(){
  const lr=streamEl.getBoundingClientRect();
  const pr=streamEl.parentElement.getBoundingClientRect();
  cvs.width=streamEl.naturalWidth||224;
  cvs.height=streamEl.naturalHeight||224;
  cvs.style.left=(lr.left-pr.left)+'px';
  cvs.style.top=(lr.top-pr.top)+'px';
  cvs.style.width=lr.width+'px';
  cvs.style.height=lr.height+'px';
}
streamEl.onload=syncCvs;
window.addEventListener('resize',syncCvs);

// stream_yolo emits UDP coordinates in the same rotated space as the MJPEG frame.
function rawToView(nx,ny){return[nx,ny];}
function viewToRaw(nx,ny){return[nx,ny];}
function normToCanvas(nx,ny){
  const p=rawToView(nx,ny);
  return[p[0]*cvs.width,p[1]*cvs.height];
}
function canvasToNorm(e){
  const r=cvs.getBoundingClientRect();
  const vx=(e.clientX-r.left)/r.width, vy=(e.clientY-r.top)/r.height;
  return viewToRaw(vx,vy);
}
function dist(a,b){return Math.hypot(a[0]-b[0],a[1]-b[1]);}

cvs.addEventListener('mousemove',e=>{if(drawing)hover=canvasToNorm(e);});
cvs.addEventListener('click',e=>{
  if(!drawing)return;
  const pt=canvasToNorm(e);
  if(drawing.pts.length>=3&&dist(pt,drawing.pts[0])<0.04){closeZone();return;}
  drawing.pts.push(pt);
});
cvs.addEventListener('contextmenu',e=>{e.preventDefault();closeZone();});

function newZone(){
  const name=document.getElementById('zname').value.trim()||('Zone '+(cfg.zones.length+1));
  drawing={name,pts:[]};
  setStatus('Click on stream to add points. Right-click or click near start to finish.');
}
function cancelZone(){drawing=null;hover=null;setStatus('');}
function closeZone(){
  if(!drawing||drawing.pts.length<3){cancelZone();return;}
  cfg.zones.push({name:drawing.name,enabled:true,points:drawing.pts});
  drawing=null;hover=null;
  renderZones();
  setStatus('Zone added.');
}

// Render loop
function frame(){
  syncCvs();
  const W=cvs.width,H=cvs.height;
  ctx.clearRect(0,0,W,H);
  cfg.zones.forEach((z,i)=>{
    if(!z.points||z.points.length<2)return;
    const col=PALETTE[i%PALETTE.length];
    ctx.beginPath();
    const[x0,y0]=normToCanvas(z.points[0][0],z.points[0][1]);
    ctx.moveTo(x0,y0);
    z.points.slice(1).forEach(p=>{const[x,y]=normToCanvas(p[0],p[1]);ctx.lineTo(x,y);});
    ctx.closePath();
    ctx.strokeStyle=col; ctx.lineWidth=2; ctx.stroke();
    ctx.fillStyle=col+'28'; ctx.fill();
    ctx.fillStyle=col; ctx.font='11px monospace';
    ctx.fillText(z.name,x0+3,y0-4);
    if(!z.enabled){ctx.fillStyle='rgba(0,0,0,0.5)';ctx.fill();}
  });
  if(drawing&&drawing.pts.length>0){
    ctx.setLineDash([5,4]);
    ctx.strokeStyle='#fff'; ctx.lineWidth=1.5;
    ctx.beginPath();
    const[x0,y0]=normToCanvas(drawing.pts[0][0],drawing.pts[0][1]);
    ctx.moveTo(x0,y0);
    drawing.pts.slice(1).forEach(p=>{const[x,y]=normToCanvas(p[0],p[1]);ctx.lineTo(x,y);});
    if(hover){const[hx,hy]=normToCanvas(hover[0],hover[1]);ctx.lineTo(hx,hy);}
    ctx.stroke(); ctx.setLineDash([]);
    drawing.pts.forEach((p,i)=>{
      const[x,y]=normToCanvas(p[0],p[1]);
      ctx.beginPath(); ctx.arc(x,y,i===0?6:3,0,2*Math.PI);
      ctx.fillStyle=i===0?'#ff0':'#fff'; ctx.fill();
    });
  }
  const now=Date.now();
  flashes=flashes.filter(d=>now-d.t<800);
  flashes.forEach(d=>{
    const a=1-(now-d.t)/800;
    const x1=d.x1/d.fw*W,y1=d.y1/d.fh*H,x2=d.x2/d.fw*W,y2=d.y2/d.fh*H;
    ctx.strokeStyle=`rgba(0,255,0,${a})`; ctx.lineWidth=2;
    ctx.strokeRect(x1,y1,x2-x1,y2-y1);
    ctx.fillStyle=`rgba(0,255,0,${a})`; ctx.font='11px monospace';
    ctx.fillText(`${d.name} ${(d.score*100).toFixed(0)}%`,x1,y1>14?y1-3:y1+13);
  });
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

function renderZones(){
  const el=document.getElementById('zlist');
  el.innerHTML='';
  if(!cfg.zones.length){el.innerHTML='<div class="hint">No zones (all area active)</div>';return;}
  cfg.zones.forEach((z,i)=>{
    const d=document.createElement('div');d.className='zi';
    const col=PALETTE[i%PALETTE.length];
    d.innerHTML=`<div class="row"><span class="zn" style="color:${col}">${z.name}</span>
      <span class="hint" style="flex:1"> ${z.points.length}pts</span>
      <button onclick="toggleZ(${i})">${z.enabled?'On':'Off'}</button>
      <button class="red" onclick="delZ(${i})">Del</button></div>`;
    el.appendChild(d);
  });
}
function toggleZ(i){cfg.zones[i].enabled=!cfg.zones[i].enabled;renderZones();}
function delZ(i){cfg.zones.splice(i,1);renderZones();}

const ALL_CLASSES=['person','vehicle','animal','bird','cat','dog','horse','sheep','cow','bear'];
function renderClasses(){
  const el=document.getElementById('fclasses');
  el.innerHTML='';
  ALL_CLASSES.forEach(c=>{
    const chk=cfg.filter_classes.includes(c);
    const d=document.createElement('label');
    d.style.cssText='display:flex;align-items:center;gap:4px;margin:2px 0;cursor:pointer';
    d.innerHTML=`<input type="checkbox" ${chk?'checked':''} onchange="toggleClass('${c}',this.checked)"> ${c}`;
    el.appendChild(d);
  });
}
function toggleClass(c,on){
  if(on&&!cfg.filter_classes.includes(c))cfg.filter_classes.push(c);
  if(!on)cfg.filter_classes=cfg.filter_classes.filter(x=>x!==c);
}

function setStatus(msg){
  document.getElementById('status').textContent=msg;
  if(msg)setTimeout(()=>{if(document.getElementById('status').textContent===msg)
    document.getElementById('status').textContent='';},4000);
}

if(document.getElementById('ip').value) connect();
</script>
</body>
</html>'''

# ---- HTTP handler ----
class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): pass

    def _cors(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET,POST,OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def do_OPTIONS(self):
        self.send_response(200)
        self._cors()
        self.end_headers()

    def do_GET(self):
        path = urlparse(self.path).path

        if path in ('/', '/index.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML)

        elif path == '/api/config':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            with config_lock:
                self.wfile.write(json.dumps(cfg).encode())

        elif path == '/api/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            status = {
                'mode': cfg.get('mode', 'ninti'),
                'yolo': _yolo_running(),
                'rtsp': _rtsp_running(),
                'tracks': len(tracks),
            }
            self.wfile.write(json.dumps(status).encode())

        elif path == '/api/events':
            rows = []
            try:
                if os.path.exists(EVENT_FILE):
                    with open(EVENT_FILE) as f:
                        lines = f.readlines()[-50:]
                    for line in lines:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            rows.append(json.loads(line))
                        except Exception:
                            pass
            except Exception as e:
                print(f'[events] read error: {e}')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            self.wfile.write(json.dumps(rows).encode())

        elif path == '/stream':
            try:
                conn = http.client.HTTPConnection('127.0.0.1', MJPEG_PORT, timeout=5)
                conn.request('GET', '/')
                resp = conn.getresponse()
                self.send_response(200)
                self.send_header('Content-Type', resp.getheader('Content-Type', 'multipart/x-mixed-replace; boundary=mjpegstream'))
                self.send_header('Cache-Control', 'no-cache')
                self._cors()
                self.end_headers()
                while True:
                    chunk = resp.read(4096)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except Exception:
                pass

        elif path == '/events':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self._cors()
            self.end_headers()
            q = Queue()
            sse_clients.append(q)
            try:
                while True:
                    try:
                        msg = q.get(timeout=20)
                        self.wfile.write(msg.encode())
                        self.wfile.flush()
                    except Empty:
                        self.wfile.write(b': ping\n\n')
                        self.wfile.flush()
            except Exception:
                pass
            finally:
                try: sse_clients.remove(q)
                except ValueError: pass

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        path = urlparse(self.path).path
        n = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(n)

        if path == '/api/config':
            try:
                new = json.loads(body)
                restart = new.pop('_restart', False)
                with config_lock:
                    cfg.update(new)
                save_cfg()
                if restart:
                    start_yolo()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self._cors()
                self.end_headers()
                self.wfile.write(b'{"ok":true}')
            except Exception as e:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(str(e).encode())

        elif path == '/api/mode':
            try:
                new = json.loads(body)
                new_mode = new.get('mode', 'ninti')
                if new_mode not in ('ninti', 'rtsp'):
                    raise ValueError(f'unknown mode: {new_mode}')
                threading.Thread(target=set_mode, args=(new_mode,), daemon=True).start()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self._cors()
                self.end_headers()
                self.wfile.write(json.dumps({'mode': new_mode}).encode())
            except Exception as e:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(str(e).encode())

        elif path == '/api/yolo/stop':
            # Gracefully stop stream_yolo (releases VPSS group) before killing sidecar
            stop_yolo()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            self.wfile.write(b'{"ok":true}')

        else:
            self.send_response(404)
            self.end_headers()


# ---- Main ----
if __name__ == '__main__':
    import signal as _signal

    def _graceful_exit(signum, frame):
        print('[main] signal received, stopping yolo before exit')
        stop_yolo()
        raise SystemExit(0)

    _signal.signal(_signal.SIGTERM, _graceful_exit)

    threading.Thread(target=udp_listener, daemon=True).start()
    threading.Thread(target=_watchdog, daemon=True).start()
    threading.Thread(target=_motion_detector, daemon=True).start()

    # Apply saved mode on startup
    saved_mode = cfg.get('mode', 'ninti')
    print(f'[main] startup mode: {saved_mode}')
    if saved_mode == 'ninti':
        _stop_rtsp()
        time.sleep(0.3)
        start_yolo()
    else:
        _stop_rtsp()
        time.sleep(0.3)
        _start_rtsp()

    server = ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), Handler)
    print(f'NintiDetect UI: http://0.0.0.0:{HTTP_PORT}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    stop_yolo()
