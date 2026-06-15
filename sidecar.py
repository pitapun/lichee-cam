#!/usr/bin/env python3
"""
NintiDetect sidecar
  - Manages stream_yolo subprocess (start / restart on threshold change)
  - Manages RTSP stack (S99gc4653rtsp start/stop), mutually exclusive with NintiDetect
  - Listens on UDP 5005 for detections, applies zone filter
  - Pushes filtered detections to browser via Server-Sent Events
  - Serves web UI on port 7778  (MJPEG still direct from :7777)
"""

import json, os, shutil, subprocess, threading, time, socket, sys, io, collections
import hashlib, base64
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse
from queue import Queue, Empty
import http.client

# ---- Paths / ports ----
CONFIG_FILE = os.environ.get('NINTI_CONFIG', '/root/ninti_config.json')
STREAM_BIN  = os.environ.get('STREAM_BIN',  '/root/stream_yolo')
MODEL       = os.environ.get('MODEL',        '/root/yolov8n_coco80.cvimodel')
RTSP_INITD  = '/etc/init.d/S99gc4653rtsp'
LD_PATH     = '/mnt/system/usr/lib:/usr/bin/lib:/root/libs_patch'
UDP_PORT    = 5005
HTTP_PORT   = 7778
MJPEG_PORT  = 7777
EVENT_DIR   = os.environ.get('NINTI_EVENT_DIR', '/root/ninti_events')
EVENT_FILE  = os.path.join(EVENT_DIR, 'events.jsonl')
EVENT_VIDEO_FPS = 25.0
EVENT_STORAGE_MAX_USED_PCT = 80.0
EVENT_STORAGE_TARGET_USED_PCT = 75.0
EVENT_STORAGE_CHECK_SEC = 300
HD_RECORD_CONTROL_FILE = '/tmp/ninti_hd_record_dir'

DEFAULT_CONFIG = {
    'mode': 'ninti',
    'threshold': 0.50,
    'zones': [],
    'filter_classes': ['person', 'vehicle', 'animal'],
    'motion_enabled': True,
    'motion_sensitivity': 20,
    # Detection zones fed to YOLO: list of {"x":int,"y":int,"size":int}.
    # size defaults to 640 (= model input); larger sizes capture a wider region
    # and are downscaled in stream_yolo before feeding the detector.
    # Max 4. Empty -> stream_yolo picks single center zone.
    'detection_zones': [],
    # When True, zone 0 becomes the "active follower": it re-centers each
    # frame on the previous frame's motion centroid (clamped to the frame).
    # UI renders this zone in orange instead of yellow.
    'active_detector': False,
    # Sensor resolution. Supported by gc4653 driver: 1280x720, 1920x1080, 2560x1440.
    'sensor_width':  1280,
    'sensor_height': 720,
    # Frame-rate cap for stream_yolo main loop. Lower = less CPU = cooler SoC.
    # 0 means uncapped (sensor max ~30fps).
    'target_fps': 15,
    'mqtt_enabled': False,
    'mqtt_host': '',
    'mqtt_port': 1883,
    'mqtt_username': '',
    'mqtt_password': '',
    'mqtt_discovery_prefix': 'homeassistant',
    'mqtt_base_topic': 'nintidetect/195',
    'mqtt_device_name': 'NintiDetect 195',
    'public_base_url': 'http://192.168.100.195:7778',
}
INFER_SIZE = 640
MAX_ZONES  = 4
SENSOR_RES_OPTIONS = [(1280, 720), (1920, 1080), (2560, 1440)]

# ---- State ----
config_lock  = threading.Lock()
sse_clients  = []          # list of Queue
yolo_proc    = None
yolo_lock    = threading.Lock()
mode_lock    = threading.Lock()
tracks_lock  = threading.Lock()
tracks       = {}
next_track_id = 1
recorders_lock = threading.Lock()
event_recorders = {}
stream_lock = threading.Condition()
latest_stream_jpg = None
latest_stream_id = 0
stream_clients = 0

PRE_BUFFER_SECS = 2.0
H264_PREROLL_SECS = 2.5
_pre_buf = collections.deque()  # (t, jpg_bytes) — rolling 3s window
_pre_buf_lock = threading.Lock()

# ---- WebSocket state ----
_ws_clients = []          # list of Queue objects
_ws_clients_lock = threading.Lock()
_ws_state = {}            # latest status fields from stream_yolo (_status UDP msgs)


def _ws_frame(data):
    """Encode payload as a WebSocket text frame (opcode=0x1, FIN set)."""
    if isinstance(data, str):
        data = data.encode('utf-8')
    length = len(data)
    hdr = bytearray([0x81])
    if length < 126:
        hdr.append(length)
    elif length < 65536:
        hdr.append(126)
        hdr.extend(length.to_bytes(2, 'big'))
    else:
        hdr.append(127)
        hdr.extend(length.to_bytes(8, 'big'))
    return bytes(hdr) + data


def _ws_broadcast(msg):
    if isinstance(msg, dict):
        msg = json.dumps(msg, separators=(',', ':'))
    frame = _ws_frame(msg)
    with _ws_clients_lock:
        dead = []
        for q in _ws_clients:
            try:
                q.put_nowait(frame)
            except Exception:
                dead.append(q)
        for q in dead:
            try: _ws_clients.remove(q)
            except ValueError: pass


def _ws_status_loop():
    while True:
        time.sleep(1.0)
        try:
            # Drive track expiry even when no detection UDP packets arrive.
            # Event recording pauses inference, so the status loop owns timeout expiry.
            _expire_tracks()
            soc_temp = None
            try:
                with open('/sys/class/thermal/thermal_zone0/temp') as _t:
                    soc_temp = round(int(_t.read().strip()) / 1000.0, 1)
            except Exception:
                pass
            with tracks_lock:
                n_tracks = len(tracks)
            with recorders_lock:
                n_events = len(event_recorders)
            status = {
                'ts': time.time(),
                'type': 'status',
                'soc_temp': soc_temp,
                'tracks': n_tracks,
                'events': n_events,
            }
            status.update(_ws_state)
            _ws_broadcast(status)
            mqtt_publish_status(status)
        except Exception:
            pass

TRACK_IOU_THRESHOLD = 0.20
TRACK_CENTER_THRESHOLD = 0.25
TRACK_CONFIRM_HITS = 2
TRACK_LOST_TIMEOUT = 2.5    # object not detected for 2.5s -> gone
TRACK_STILL_TIMEOUT = 5.0   # object not moving for 5s -> gone
RECORD_STILL_TIMEOUT = 2.5  # during recording, stationary object ends the clip
TRACK_MOVE_THRESHOLD = 0.04 # min center shift (normalised) to count as movement
STATIONARY_SUPPRESS_ABSENT_TIMEOUT = 60.0
STATIONARY_SUPPRESS_CENTER_THRESHOLD = 0.20
STATIONARY_SUPPRESS_IOU_THRESHOLD = 0.05
MIN_RECORD_SECS = 3.0       # minimum recording duration after confirmation
MAX_RECORD_SECS = 12.0      # hard cap for a single saved event

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

def public_cfg():
    with config_lock:
        out = cfg.copy()
    pw = out.pop('mqtt_password', '')
    out['mqtt_has_password'] = bool(pw)
    return out

# ---- Minimal MQTT publisher (MQTT 3.1.1, QoS 0, stdlib only) ----
mqtt_lock = threading.Lock()
mqtt_state = {'last_error': '', 'last_pub': 0, 'discovery_sent': False}
_mqtt_last_status = 0

def _mqtt_enabled_cfg():
    with config_lock:
        return {
            'enabled': bool(cfg.get('mqtt_enabled')),
            'host': str(cfg.get('mqtt_host') or '').strip(),
            'port': int(cfg.get('mqtt_port') or 1883),
            'username': str(cfg.get('mqtt_username') or ''),
            'password': str(cfg.get('mqtt_password') or ''),
            'prefix': str(cfg.get('mqtt_discovery_prefix') or 'homeassistant').strip('/'),
            'base': str(cfg.get('mqtt_base_topic') or 'nintidetect/195').strip('/'),
            'name': str(cfg.get('mqtt_device_name') or 'NintiDetect 195'),
        }

def _mqtt_rl(n):
    out = bytearray()
    while True:
        b = n % 128
        n //= 128
        if n:
            b |= 0x80
        out.append(b)
        if not n:
            return bytes(out)

def _mqtt_str(s):
    b = str(s).encode('utf-8')
    return len(b).to_bytes(2, 'big') + b

def _mqtt_connect_packet(client_id, username, password):
    flags = 0x02
    payload = _mqtt_str(client_id)
    if username:
        flags |= 0x80
        payload += _mqtt_str(username)
    if password:
        flags |= 0x40
        payload += _mqtt_str(password)
    vh = _mqtt_str('MQTT') + bytes([4, flags]) + (30).to_bytes(2, 'big')
    body = vh + payload
    return bytes([0x10]) + _mqtt_rl(len(body)) + body

def _mqtt_publish_packet(topic, payload, retain=False):
    if isinstance(payload, dict):
        payload = json.dumps(payload, separators=(',', ':'))
    body = _mqtt_str(topic) + str(payload).encode('utf-8')
    return bytes([0x31 if retain else 0x30]) + _mqtt_rl(len(body)) + body

def _mqtt_publish_many(messages):
    mc = _mqtt_enabled_cfg()
    if not mc['enabled'] or not mc['host']:
        return False
    with mqtt_lock:
        try:
            s = socket.create_connection((mc['host'], mc['port']), timeout=3)
            try:
                cid = 'nintidetect-' + socket.gethostname()
                s.sendall(_mqtt_connect_packet(cid, mc['username'], mc['password']))
                resp = s.recv(4)
                if len(resp) < 4 or resp[0] != 0x20 or resp[3] != 0:
                    raise RuntimeError(f'connect refused {resp!r}')
                for topic, payload, retain in messages:
                    s.sendall(_mqtt_publish_packet(topic, payload, retain))
                try:
                    s.shutdown(socket.SHUT_WR)
                except Exception:
                    pass
                time.sleep(0.05)
                mqtt_state['last_error'] = ''
                mqtt_state['last_pub'] = time.time()
                return True
            finally:
                try: s.close()
                except Exception: pass
        except Exception as e:
            mqtt_state['last_error'] = str(e)
            return False

def _mqtt_device(mc):
    return {
        'identifiers': ['nintidetect_195'],
        'name': mc['name'],
        'manufacturer': 'Thylation',
        'model': 'LicheeRV Nano NintiDetect',
    }

def _mqtt_discovery_messages():
    mc = _mqtt_enabled_cfg()
    base, prefix = mc['base'], mc['prefix']
    dev = _mqtt_device(mc)
    common = {'device': dev, 'availability_topic': f'{base}/availability'}
    specs = [
        ('sensor', 'temperature', {
            'name': 'Temperature', 'state_topic': f'{base}/status',
            'value_template': '{{ value_json.soc_temp }}',
            'unit_of_measurement': '°C', 'device_class': 'temperature',
        }),
        ('sensor', 'fps', {
            'name': 'FPS', 'state_topic': f'{base}/status',
            'value_template': '{{ value_json.fps }}',
            'unit_of_measurement': 'fps',
        }),
        ('sensor', 'tracks', {
            'name': 'Tracks', 'state_topic': f'{base}/status',
            'value_template': '{{ value_json.tracks }}',
        }),
        ('sensor', 'events', {
            'name': 'Active Events', 'state_topic': f'{base}/status',
            'value_template': '{{ value_json.events }}',
        }),
        ('sensor', 'last_event', {
            'name': 'Last Event', 'state_topic': f'{base}/event',
            'value_template': '{{ value_json.name }}',
            'json_attributes_topic': f'{base}/event',
        }),
        ('sensor', 'recent_events', {
            'name': 'Recent Events', 'state_topic': f'{base}/recent_events',
            'value_template': '{{ value_json.latest_name }}',
            'json_attributes_topic': f'{base}/recent_events',
        }),
        ('sensor', 'last_person_event', {
            'name': 'Last Person Event', 'state_topic': f'{base}/last/person',
            'value_template': '{{ value_json.name }}',
            'json_attributes_topic': f'{base}/last/person',
        }),
        ('sensor', 'last_vehicle_event', {
            'name': 'Last Vehicle Event', 'state_topic': f'{base}/last/vehicle',
            'value_template': '{{ value_json.name }}',
            'json_attributes_topic': f'{base}/last/vehicle',
        }),
        ('sensor', 'last_other_event', {
            'name': 'Last Other Event', 'state_topic': f'{base}/last/other',
            'value_template': '{{ value_json.name }}',
            'json_attributes_topic': f'{base}/last/other',
        }),
        ('binary_sensor', 'detector', {
            'name': 'Detector', 'state_topic': f'{base}/status',
            'value_template': "{{ 'ON' if value_json.yolo else 'OFF' }}",
            'payload_on': 'ON', 'payload_off': 'OFF',
        }),
        ('binary_sensor', 'recording', {
            'name': 'Recording', 'state_topic': f'{base}/status',
            'value_template': "{{ 'ON' if value_json.events|int > 0 else 'OFF' }}",
            'payload_on': 'ON', 'payload_off': 'OFF',
        }),
    ]
    msgs = [(f'{base}/availability', 'online', True)]
    for domain, key, payload in specs:
        payload.update(common)
        payload['unique_id'] = f'nintidetect_195_{key}'
        payload['object_id'] = f'nintidetect_195_{key}'
        msgs.append((f'{prefix}/{domain}/nintidetect_195/{key}/config', payload, True))
    msgs.append((f'{prefix}/sensor/nintidetect_195/last_animal_event/config', '', True))
    return msgs

def mqtt_publish_discovery(force=False):
    if not force and mqtt_state.get('discovery_sent'):
        return
    if _mqtt_publish_many(_mqtt_discovery_messages()):
        mqtt_state['discovery_sent'] = True

def mqtt_publish_status(status, force=False):
    global _mqtt_last_status
    mc = _mqtt_enabled_cfg()
    if not mc['enabled'] or not mc['host']:
        return
    now = time.time()
    if not force and now - _mqtt_last_status < 5:
        return
    _mqtt_last_status = now
    mqtt_publish_discovery()
    payload = status.copy()
    payload['mode'] = cfg.get('mode', 'ninti')
    payload['yolo'] = _yolo_running()
    payload['rtsp'] = _rtsp_running()
    _mqtt_publish_many([(f"{mc['base']}/status", payload, False)])

def mqtt_publish_event(rec):
    mc = _mqtt_enabled_cfg()
    if not mc['enabled'] or not mc['host']:
        return
    mqtt_publish_discovery()
    _mqtt_publish_many([(f"{mc['base']}/event", rec, False)])

def _abs_url(path):
    if not path:
        return ''
    if str(path).startswith(('http://', 'https://')):
        return path
    base = str(cfg.get('public_base_url') or '').rstrip('/')
    return base + str(path)

def _recent_events_payload(limit=3):
    rows = []
    try:
        if os.path.exists(EVENT_FILE):
            with open(EVENT_FILE) as f:
                lines = f.readlines()[-100:]
            for line in lines:
                try:
                    rec = json.loads(line.strip())
                except Exception:
                    continue
                if not rec.get('video_url') and not rec.get('thumbnail_url'):
                    continue
                thumb = rec.get('thumbnail_url')
                if not thumb and rec.get('video_url'):
                    thumb = rec.get('video_url').replace('event.mp4', 'best_frame.jpg')
                item = {
                    'track_id': rec.get('track_id'),
                    'cat': rec.get('cat'),
                    'name': rec.get('name') or rec.get('cat') or 'event',
                    'first_seen': rec.get('first_seen'),
                    'duration_s': rec.get('duration_s'),
                    'best_score': rec.get('best_score'),
                    'thumbnail_url': thumb,
                    'thumbnail_url_abs': _abs_url(thumb),
                    'video_url': rec.get('video_url'),
                    'video_url_abs': _abs_url(rec.get('video_url')),
                    'event_meta_url': rec.get('event_meta_url'),
                }
                rows.append(item)
    except Exception:
        pass
    recent = list(reversed(rows[-limit:]))
    return {
        'count': len(recent),
        'latest_name': recent[0].get('name') if recent else 'none',
        'events': recent,
        'ts': time.time(),
    }

def _event_item(rec):
    thumb = rec.get('thumbnail_url')
    if not thumb and rec.get('video_url'):
        thumb = rec.get('video_url').replace('event.mp4', 'best_frame.jpg')
    return {
        'track_id': rec.get('track_id'),
        'cat': rec.get('cat'),
        'name': rec.get('name') or rec.get('cat') or 'none',
        'first_seen': rec.get('first_seen'),
        'duration_s': rec.get('duration_s'),
        'best_score': rec.get('best_score'),
        'thumbnail_url': thumb,
        'thumbnail_url_abs': _abs_url(thumb),
        'video_url': rec.get('video_url'),
        'video_url_abs': _abs_url(rec.get('video_url')),
        'event_meta_url': rec.get('event_meta_url'),
        'ts': time.time(),
    }

def _last_event_by_category_payload():
    latest = {}
    try:
        if os.path.exists(EVENT_FILE):
            with open(EVENT_FILE) as f:
                lines = f.readlines()[-300:]
            for line in lines:
                try:
                    rec = json.loads(line.strip())
                except Exception:
                    continue
                cat = rec.get('cat')
                cat_key = cat if cat in ('person', 'vehicle') else 'other'
                if not rec.get('video_url') and not rec.get('thumbnail_url'):
                    continue
                item = _event_item(rec)
                item['cat'] = cat_key
                latest[cat_key] = item
    except Exception:
        pass
    empty = lambda cat: {
        'cat': cat, 'name': 'none', 'thumbnail_url': '', 'thumbnail_url_abs': '',
        'video_url': '', 'video_url_abs': '', 'ts': time.time(),
    }
    return {cat: latest.get(cat) or empty(cat) for cat in ('person', 'vehicle', 'other')}

def mqtt_publish_recent_events():
    mc = _mqtt_enabled_cfg()
    if not mc['enabled'] or not mc['host']:
        return
    mqtt_publish_discovery()
    by_cat = _last_event_by_category_payload()
    _mqtt_publish_many([
        (f"{mc['base']}/recent_events", _recent_events_payload(3), True),
        (f"{mc['base']}/last/person", by_cat['person'], True),
        (f"{mc['base']}/last/vehicle", by_cat['vehicle'], True),
        (f"{mc['base']}/last/other", by_cat['other'], True),
        (f"{mc['base']}/last/animal", '', True),
    ])

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

stationary_suppressions = []

def _prune_stationary_suppressions(now):
    global stationary_suppressions
    stationary_suppressions = [
        s for s in stationary_suppressions
        if now - s.get('last_seen', now) <= STATIONARY_SUPPRESS_ABSENT_TIMEOUT
    ]

def _stationary_match(det, sup):
    same_cat = sup.get('cat') and sup.get('cat') == det.get('cat')
    same_cls = sup.get('cls') is not None and sup.get('cls') == det.get('cls')
    if not (same_cat or same_cls):
        return False
    dist = _center_dist(det, sup.get('last_det', det))
    iou = _iou(_bbox(det), sup.get('bbox', _bbox(det)))
    return dist <= STATIONARY_SUPPRESS_CENTER_THRESHOLD or iou >= STATIONARY_SUPPRESS_IOU_THRESHOLD

def _remember_stationary_suppression(tr, now):
    _prune_stationary_suppressions(now)
    sup = {
        'cls': tr.get('cls'),
        'cat': tr.get('cat'),
        'name': tr.get('name'),
        'bbox': tr.get('bbox'),
        'last_det': tr.get('last_det', {}).copy(),
        'last_seen': now,
        'source_track_id': tr.get('id'),
    }
    for i, existing in enumerate(stationary_suppressions):
        if _stationary_match(sup['last_det'], existing):
            stationary_suppressions[i] = sup
            return
    stationary_suppressions.append(sup)
    print(f'[events] stationary suppress track {tr.get("id")} {tr.get("name")}', flush=True)

def _suppressed_stationary_detection(det, now):
    _prune_stationary_suppressions(now)
    for sup in stationary_suppressions:
        if _stationary_match(det, sup):
            sup['bbox'] = _bbox(det)
            sup['last_det'] = det.copy()
            sup['last_seen'] = now
            return sup
    return None

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
            'duration_s': round(now - tr.get('first_seen', now), 3),
            'hits': tr.get('hits', 0),
            'best_score': tr.get('best_score', 0),
            'bbox': tr.get('bbox'),
            't': now,
        }
        if tr.get('video_url'):
            rec['video_url'] = tr.get('video_url')
        if tr.get('thumbnail_url'):
            rec['thumbnail_url'] = tr.get('thumbnail_url')
        if tr.get('event_dir'):
            rec['event_dir'] = tr.get('event_dir')
        if tr.get('event_meta_url'):
            rec['event_meta_url'] = tr.get('event_meta_url')
        if tr.get('frames_url'):
            rec['frames_url'] = tr.get('frames_url')
        if tr.get('num_frames') is not None:
            rec['num_frames'] = tr.get('num_frames')
        if tr.get('record_fps') is not None:
            rec['record_fps'] = tr.get('record_fps')
        with open(EVENT_FILE, 'a') as f:
            f.write(json.dumps(rec, separators=(',', ':')) + '\n')
        mqtt_publish_event(rec)
        mqtt_publish_recent_events()
    except Exception as e:
        print(f'[events] write error: {e}')

def _has_event_recorders():
    with recorders_lock:
        return bool(event_recorders)

def _start_event_video(tr, now):
    tid = tr['id']
    _cleanup_event_storage()
    with recorders_lock:
        if tid in event_recorders:
            return
        os.makedirs(EVENT_DIR, exist_ok=True)
        stamp = int(now)
        tmp_dir = os.path.join(EVENT_DIR, f'.track_{tid:05d}_recording_{stamp}')
        frames_dir = os.path.join(tmp_dir, 'frames')
        os.makedirs(frames_dir, exist_ok=True)
        actual_fps = float(cfg.get('target_fps') or EVENT_VIDEO_FPS)

        # Write pre-buffer frames (last PRE_BUFFER_SECS before trigger)
        pre_idx = 0
        pre_last_t = now - (1.0 / actual_fps)
        with _pre_buf_lock:
            cutoff = now - PRE_BUFFER_SECS
            pre_frames = [(t, j) for (t, j) in _pre_buf if t >= cutoff]
        for (t, jpg_bytes) in pre_frames:
            path = os.path.join(frames_dir, f'frame_{pre_idx:05d}.jpg')
            try:
                with open(path, 'wb') as f:
                    f.write(jpg_bytes)
                pre_idx += 1
                pre_last_t = t
            except Exception as e:
                print(f'[events] pre-buf write error: {e}', flush=True)

        event_recorders[tid] = {
            'track_id': tid,
            'tmp_dir': tmp_dir,
            'frames_dir': frames_dir,
            'last_frame_t': pre_last_t,
            'frame_count': pre_idx,
            'fps': actual_fps,
            'started_at': pre_frames[0][0] if pre_frames else now,
        }
        try:
            with open(HD_RECORD_CONTROL_FILE, 'w') as f:
                f.write(frames_dir)
        except Exception as e:
            print(f'[events] hd control write error: {e}', flush=True)
    print(f'[events] recording start track {tid} pre_buf={pre_idx}fr', flush=True)

def _record_event_frame(jpg, now):
    if os.path.exists(HD_RECORD_CONTROL_FILE):
        return
    writes = []
    with recorders_lock:
        for rec in event_recorders.values():
            interval = 1.0 / rec['fps']
            catchup = 0
            while now - rec['last_frame_t'] >= interval and catchup < 16:
                idx = rec['frame_count']
                rec['frame_count'] += 1
                rec['last_frame_t'] += interval
                writes.append(os.path.join(rec['frames_dir'], f'frame_{idx:05d}.jpg'))
                catchup += 1
    for path in writes:
        try:
            with open(path, 'wb') as f:
                f.write(jpg)
        except Exception as e:
            print(f'[events] frame write error: {e}', flush=True)

def _record_event_frames_from_buffer(buf):
    while True:
        s = buf.find(b'\xff\xd8')
        if s < 0:
            return b''
        e = buf.find(b'\xff\xd9', s + 2)
        if e < 0:
            return buf[s:]
        jpg = buf[s:e + 2]
        buf = buf[e + 2:]
        _record_event_frame(jpg, time.time())

def _publish_stream_frame(jpg):
    global latest_stream_jpg, latest_stream_id
    with stream_lock:
        latest_stream_jpg = jpg
        latest_stream_id += 1
        stream_lock.notify_all()

def _push_pre_buf(jpg, t):
    with _pre_buf_lock:
        _pre_buf.append((t, jpg))
        cutoff = t - (PRE_BUFFER_SECS + 1.0)
        while _pre_buf and _pre_buf[0][0] < cutoff:
            _pre_buf.popleft()

def _stream_interest():
    with stream_lock:
        clients = stream_clients
    if clients > 0:
        return True
    if _has_event_recorders() and not os.path.exists(HD_RECORD_CONTROL_FILE):
        return True
    return False

def _disk_used_pct(path):
    os.makedirs(path, exist_ok=True)
    usage = shutil.disk_usage(path)
    return (usage.used / max(1, usage.total)) * 100.0

def _valid_event_video_url(url):
    if not url or not str(url).startswith('/event-video/'):
        return False
    rel = os.path.normpath(str(url)[len('/event-video/'):]).lstrip('/')
    full = os.path.abspath(os.path.join(EVENT_DIR, rel))
    root = os.path.abspath(EVENT_DIR)
    return full.startswith(root + os.sep) and os.path.exists(full)

def _rewrite_event_index_without_deleted_videos():
    if not os.path.exists(EVENT_FILE):
        return
    kept = []
    try:
        with open(EVENT_FILE) as f:
            for line in f:
                raw = line.strip()
                if not raw:
                    continue
                try:
                    rec = json.loads(raw)
                except Exception:
                    continue
                if rec.get('video_url') and not _valid_event_video_url(rec.get('video_url')):
                    continue
                kept.append(json.dumps(rec, separators=(',', ':')))
        tmp = EVENT_FILE + '.tmp'
        with open(tmp, 'w') as f:
            if kept:
                f.write('\n'.join(kept) + '\n')
        os.replace(tmp, EVENT_FILE)
    except Exception as e:
        print(f'[storage] index rewrite error: {e}', flush=True)

def _cleanup_event_storage():
    try:
        os.makedirs(EVENT_DIR, exist_ok=True)
        now = time.time()
        for name in os.listdir(EVENT_DIR):
            if not name.startswith('.track_') or '_recording_' not in name:
                continue
            path = os.path.join(EVENT_DIR, name)
            try:
                if os.path.isdir(path) and now - os.path.getmtime(path) > 3600:
                    shutil.rmtree(path)
                    print(f'[storage] removed stale temp {path}', flush=True)
            except Exception:
                pass

        used = _disk_used_pct(EVENT_DIR)
        if used < EVENT_STORAGE_MAX_USED_PCT:
            return

        event_dirs = []
        for name in os.listdir(EVENT_DIR):
            if not (name.startswith('event_') and '_save_' in name):
                continue
            path = os.path.join(EVENT_DIR, name)
            if os.path.isdir(path):
                event_dirs.append((os.path.getmtime(path), path))
        event_dirs.sort()
        deleted = 0
        for _mtime, path in event_dirs:
            if _disk_used_pct(EVENT_DIR) <= EVENT_STORAGE_TARGET_USED_PCT:
                break
            shutil.rmtree(path)
            deleted += 1
            print(f'[storage] deleted old event {path}', flush=True)
        if deleted:
            _rewrite_event_index_without_deleted_videos()
            print(f'[storage] cleanup done used={_disk_used_pct(EVENT_DIR):.1f}%', flush=True)
    except Exception as e:
        print(f'[storage] cleanup error: {e}', flush=True)

def _storage_cleanup_loop():
    while True:
        _cleanup_event_storage()
        time.sleep(EVENT_STORAGE_CHECK_SEC)

def _effective_hls_fps(fps=None):
    if fps is None:
        with config_lock:
            fps = cfg.get('target_fps', 0) or 5
    try:
        fps = int(float(fps))
    except Exception:
        fps = 5
    return max(1, min(fps, 5))

def _count_h264_frames(h264_in):
    try:
        out = subprocess.check_output([
            'ffprobe', '-v', 'error', '-f', 'h264', '-count_frames',
            '-select_streams', 'v:0',
            '-show_entries', 'stream=nb_read_frames',
            '-of', 'default=nw=1:nk=1', h264_in
        ], stderr=subprocess.DEVNULL, timeout=10).decode().strip()
        for line in out.splitlines():
            line = line.strip()
            if line.isdigit():
                return int(line)
    except Exception:
        pass
    return None

def _h264_remux_fps(h264_in, duration_s, fallback_fps):
    frames = _count_h264_frames(h264_in)
    if frames and duration_s > 0.1:
        return max(0.1, min(60.0, frames / duration_s))
    return fallback_fps

def _fmt_fps(fps):
    try:
        fps = float(fps)
    except Exception:
        fps = float(EVENT_VIDEO_FPS)
    return str(int(fps)) if fps == int(fps) else f'{fps:.3f}'.rstrip('0').rstrip('.')

def _run_ffmpeg(frames_dir, out_mp4, fps, h264_in=None, h264_fps=None):
    if h264_in and os.path.exists(h264_in):
        rfps = _fmt_fps(h264_fps or _effective_hls_fps(fps))
        cmds = [
            ['ffmpeg', '-y', '-loglevel', 'error', '-fflags', '+genpts',
             '-framerate', rfps, '-i', h264_in, '-c:v', 'copy',
             out_mp4],
            ['ffmpeg', '-y', '-loglevel', 'error', '-fflags', '+genpts',
             '-framerate', rfps, '-i', h264_in, '-c:v', 'mpeg4', '-pix_fmt', 'yuv420p',
             out_mp4],
        ]
        for cmd in cmds:
            try:
                subprocess.check_call(cmd)
                return os.path.exists(out_mp4)
            except Exception:
                pass
    pattern = os.path.join(frames_dir, 'frame_%05d.jpg')
    rfps = _fmt_fps(fps)
    cmds = [
        ['ffmpeg', '-y', '-loglevel', 'error', '-framerate', rfps,
         '-i', pattern, '-c:v', 'libx264', '-pix_fmt', 'yuv420p', out_mp4],
        ['ffmpeg', '-y', '-loglevel', 'error', '-framerate', rfps,
         '-i', pattern, '-c:v', 'mpeg4', '-pix_fmt', 'yuv420p', out_mp4],
    ]
    for cmd in cmds:
        try:
            subprocess.check_call(cmd)
            return os.path.exists(out_mp4)
        except Exception:
            pass
    return False

def _wait_hd_record_done(frames_dir, timeout=15.0):
    done = os.path.join(frames_dir, '.hd_done')
    deadline = time.time() + timeout
    while time.time() < deadline:
        if os.path.exists(done):
            return True
        time.sleep(0.1)
    return False

def _finish_event_video(tr, save, now):
    tid = tr['id']
    with recorders_lock:
        rec = event_recorders.pop(tid, None)
    if not rec:
        return None
    try:
        if os.path.exists(HD_RECORD_CONTROL_FILE):
            with open(HD_RECORD_CONTROL_FILE) as f:
                active_dir = f.read().strip()
            if active_dir == rec.get('frames_dir'):
                os.unlink(HD_RECORD_CONTROL_FILE)
                _wait_hd_record_done(rec.get('frames_dir'))
    except Exception:
        pass
    tmp_dir = rec['tmp_dir']
    if not save:
        try:
            subprocess.call(['rm', '-rf', tmp_dir])
        except Exception:
            pass
        print(f'[events] recording discarded track {tid}', flush=True)
        return None

    try:
        actual_frames = [p for p in os.listdir(rec['frames_dir'])
                         if p.startswith('frame_') and p.endswith('.jpg')]
    except Exception:
        actual_frames = []
    h264_tmp = os.path.join(tmp_dir, 'event.h264')
    if not actual_frames and not os.path.exists(h264_tmp):
        subprocess.call(['rm', '-rf', tmp_dir])
        return None

    stamp = int(now)
    event_name = f'event_{tid:05d}_save_{stamp}'
    event_dir = os.path.join(EVENT_DIR, event_name)
    if os.path.exists(event_dir):
        subprocess.call(['rm', '-rf', event_dir])
    os.rename(tmp_dir, event_dir)
    frames_dir = os.path.join(event_dir, 'frames')
    h264_in = os.path.join(event_dir, 'event.h264')
    frame_files = sorted([p for p in os.listdir(frames_dir) if p.endswith('.jpg')])
    thumb_ok = False
    if frame_files:
        best = os.path.join(frames_dir, frame_files[len(frame_files)//2])
        thumb_ok = subprocess.call(['cp', best, os.path.join(event_dir, 'best_frame.jpg')]) == 0
    out_mp4 = os.path.join(event_dir, 'event.mp4')
    if os.path.exists(h264_in):
        h264_duration = max(0.1, now - rec.get('started_at', now) + H264_PREROLL_SECS)
        record_fps = _h264_remux_fps(h264_in, h264_duration, _effective_hls_fps(rec['fps']))
    else:
        record_fps = rec['fps']
    ok = _run_ffmpeg(frames_dir, out_mp4, rec['fps'], h264_in, record_fps)
    if ok and not thumb_ok:
        thumb_ok = subprocess.call([
            'ffmpeg', '-y', '-loglevel', 'error', '-i', out_mp4,
            '-frames:v', '1', os.path.join(event_dir, 'best_frame.jpg')
        ]) == 0
    event_url = f'/event-video/{event_name}'
    meta = {
        'track_id': tid,
        'cat': tr.get('cat'),
        'cls': tr.get('cls'),
        'name': tr.get('name'),
        'first_seen': tr.get('first_seen'),
        'last_seen': tr.get('last_seen'),
        'duration_s': round(now - tr.get('first_seen', now), 3),
        'hits': tr.get('hits', 0),
        'best_score': tr.get('best_score', 0),
        'record_fps': record_fps,
        'num_frames': len(frame_files),
        'video_ok': ok,
        'event_dir': event_url,
        'event_meta_url': f'{event_url}/event.json',
        'frames_url': f'{event_url}/frames/',
        'video_url': f'{event_url}/event.mp4' if ok else None,
        'thumbnail_url': f'{event_url}/best_frame.jpg' if thumb_ok else None,
    }
    with open(os.path.join(event_dir, 'event.json'), 'w') as f:
        json.dump(meta, f, indent=2)
    print(f'[events] recording saved track {tid} -> {event_dir}', flush=True)
    _cleanup_event_storage()
    return meta

def _expire_tracks(now=None):
    if now is None:
        now = time.time()
    # Snapshot which tracks have active recorders before taking tracks_lock,
    # so _finish_event_video can safely acquire recorders_lock later.
    with recorders_lock:
        recording_ids = set(event_recorders.keys())
    expired = []
    with tracks_lock:
        for tid, tr in list(tracks.items()):
            recording = tid in recording_ids
            if recording:
                lost  = now - tr.get('last_seen', now) > TRACK_LOST_TIMEOUT
                still = now - tr.get('last_moved', now) > RECORD_STILL_TIMEOUT
            else:
                lost  = now - tr.get('last_seen',  now) > TRACK_LOST_TIMEOUT
                still = now - tr.get('last_moved', now) > TRACK_STILL_TIMEOUT
            maxed = tr.get('confirmed') and now - tr.get('first_seen', now) >= MAX_RECORD_SECS
            if (lost or still or maxed) and now >= tr.get('min_record_until', 0):
                expired.append(tid)
        for tid in expired:
            tr = tracks.pop(tid, {})
            duration = now - tr.get('first_seen', now)
            fps = float(cfg.get('target_fps') or EVENT_VIDEO_FPS)
            min_dur = 2.0 / fps  # discard if shorter than 2 frames
            if tr.get('confirmed') and duration >= min_dur:
                # only write events for confirmed tracks with sufficient duration
                video_meta = _finish_event_video(tr, True, now)
                if video_meta:
                    tr['video_url'] = video_meta.get('video_url')
                    tr['thumbnail_url'] = video_meta.get('thumbnail_url')
                    tr['event_dir'] = video_meta.get('event_dir')
                    tr['event_meta_url'] = video_meta.get('event_meta_url')
                    tr['frames_url'] = video_meta.get('frames_url')
                    tr['num_frames'] = video_meta.get('num_frames')
                    tr['record_fps'] = video_meta.get('record_fps')
                _write_event('end', tr, now)
                _remember_stationary_suppression(tr, now)
                msg = 'data: ' + json.dumps({'_save': True, 'track_id': tid,
                                              'name': tr.get('name', '?'),
                                              'hits': tr.get('hits', 0),
                                              'duration_s': round(duration, 1),
                                              'video_url': tr.get('video_url'),
                                              'thumbnail_url': tr.get('thumbnail_url'),
                                              'event_meta_url': tr.get('event_meta_url')}) + '\n\n'
            else:
                _finish_event_video(tr, False, now)
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
            sup = _suppressed_stationary_detection(det, now)
            if sup:
                det['track_id'] = sup.get('source_track_id')
                det['track_hits'] = 0
                det['track_confirmed'] = False
                det['track_suppressed'] = True
                return det

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
            _start_event_video(tr, now)
            tr['min_record_until'] = now + MIN_RECORD_SECS

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

# ---- HLS ffmpeg lifecycle ----
HLS_FIFO = '/tmp/hls_feed.h264'
HLS_DIR  = '/tmp/hls'
HLS_ENABLED = True
_hls_proc = None
_hls_lock = threading.Lock()

def _start_hls_ffmpeg():
    global _hls_proc
    if not HLS_ENABLED:
        return
    hls_fps = _effective_hls_fps()
    os.makedirs(HLS_DIR, exist_ok=True)
    if not os.path.exists(HLS_FIFO):
        os.mkfifo(HLS_FIFO)
    time.sleep(1)
    with _hls_lock:
        if _hls_proc and _hls_proc.poll() is None:
            _hls_proc.terminate()
            try: _hls_proc.wait(3)
            except: _hls_proc.kill()
        _hls_proc = subprocess.Popen([
            'ffmpeg', '-y', '-loglevel', 'error',
            '-use_wallclock_as_timestamps', '1',
            '-fflags', '+genpts',
            '-f', 'h264', '-r', str(hls_fps), '-i', HLS_FIFO,
            '-c:v', 'copy',
            '-f', 'hls',
            '-hls_time', '2',
            '-hls_list_size', '5',
            '-hls_flags', 'delete_segments+append_list+omit_endlist+split_by_time',
            f'{HLS_DIR}/live.m3u8',
        ], stdout=subprocess.DEVNULL, stderr=open('/tmp/hls_ffmpeg.log', 'w'))
        print(f'[hls] ffmpeg started pid={_hls_proc.pid}')

def _stop_hls_ffmpeg():
    global _hls_proc
    with _hls_lock:
        if _hls_proc:
            _hls_proc.terminate()
            try: _hls_proc.wait(5)
            except: _hls_proc.kill()
            _hls_proc = None
    for f in os.listdir(HLS_DIR) if os.path.isdir(HLS_DIR) else []:
        try: os.remove(os.path.join(HLS_DIR, f))
        except: pass
    print('[hls] ffmpeg stopped')

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
        # Replace musl's mallocng (heavy per-thread mmap fragmentation under
        # opencv + cvi-sdk churn -> ~110 KB/min VmData creep, OOM in hours)
        # with jemalloc, which returns idle pages to the OS via madvise and
        # coalesces thread arenas. Only injected into stream_yolo, not into
        # sidecar python3 or ffmpeg, to keep the blast radius narrow.
        if os.path.exists('/root/libs_patch/libjemalloc.so.2'):
            env['LD_PRELOAD'] = '/root/libs_patch/libjemalloc.so.2'
            env['MALLOC_CONF'] = 'retain:false,dirty_decay_ms:0,muzzy_decay_ms:0'
        # GC4653 sensor floor is ~2.75fps; clamp at 3 even if user picks lower
        # (target_frame_us sleep in main.cpp still honours the user value).
        _tfps_env = int(cfg.get('target_fps', 0) or 0)
        if _tfps_env > 0:
            env['YOLO_CAP_FPS'] = str(max(3, min(30, _tfps_env)))
        # Sensor driver selection. Default = GC4653; set sensor_driver: "OS04A10"
        # in config on boards where GC4653 driver causes VI permission errors.
        if cfg.get('sensor_driver'):
            env['YOLO_SNS'] = str(cfg['sensor_driver'])
        thresh = str(cfg.get('threshold', 0.45))
        sw = int(cfg.get('sensor_width',  1280))
        sh = int(cfg.get('sensor_height', 720))
        if (sw, sh) not in SENSOR_RES_OPTIONS:
            sw, sh = 1280, 720
        # Build --zones "x,y;x,y;..." (clamped to frame). Empty list -> auto-
        # populate a single center zone and persist, so the UI yellow box
        # matches what stream_yolo actually crops.
        raw_zones = cfg.get('detection_zones', []) or []
        if not raw_zones:
            raw_zones = [{'x': max(0, (sw - INFER_SIZE)//2),
                          'y': max(0, (sh - INFER_SIZE)//2),
                          'size': INFER_SIZE}]
            with config_lock:
                cfg['detection_zones'] = raw_zones
            save_cfg()
        clamped = []
        for z in raw_zones[:MAX_ZONES]:
            try:
                x = int(z.get('x', 0)); y = int(z.get('y', 0))
                size = int(z.get('size', INFER_SIZE))
            except Exception:
                continue
            size = max(INFER_SIZE, size)  # enforce minimum only; zone may exceed frame
            clamped.append((x, y, size))
        args = [STREAM_BIN, MODEL, '80', str(INFER_SIZE), thresh,
                str(UDP_PORT), str(sw), str(sh)]
        if clamped:
            args += ['--zones', ';'.join(f'{x},{y},{s}' for x, y, s in clamped)]
        if cfg.get('active_detector', False):
            args += ['--active-detector']
        tfps = int(cfg.get('target_fps', 0) or 0)
        if tfps > 0:
            args += ['--fps', str(tfps)]
        msen = float(cfg.get('motion_sensitivity', 20))
        args += ['--motion-thresh', f'{msen / 5.0:.2f}']
        print(f'[yolo] starting threshold={thresh} sensor={sw}x{sh} '
              f'zones={clamped} active={cfg.get("active_detector",False)} '
              f'fps_cap={tfps or "off"}')
        _stop_hls_ffmpeg()
        yolo_log = open('/tmp/stream_yolo.log', 'a')
        yolo_log.write(f'\n[yolo] exec start ts={time.strftime("%Y-%m-%d %H:%M:%S")}\n')
        yolo_log.flush()
        yolo_proc = subprocess.Popen(args, env=env, stdout=yolo_log, stderr=yolo_log)
        if HLS_ENABLED:
            threading.Thread(target=_start_hls_ffmpeg, daemon=True).start()

def start_yolo():
    threading.Thread(target=_start_yolo_inner, daemon=True).start()

def stop_yolo():
    global yolo_proc
    _stop_hls_ffmpeg()
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
            proc = yolo_proc
            pid = proc.pid if proc else None
            exit_code = proc.poll() if proc else None
        if proc is None:
            _dstate_count = 0
            continue
        if exit_code is not None:
            # Process died (OOM, crash). VPSS group + ION buffers are
            # leaked in kernel state and a respawn gets EBUSY on the
            # sensor, so the only reliable recovery is reboot. (Verified
            # 2026-06-12: post-OOM respawn -> CVI_VI_EnableChn c002800c.)
            print(f'[watchdog] stream_yolo pid {pid} died (exit={exit_code}); rebooting')
            _sp.call(['reboot', '-f'])
            time.sleep(60)
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

# ---- RSS logger (memory growth tracking) ----
def _rss_logger():
    """Sample stream_yolo VmRSS every 30s into /tmp/stream_yolo_rss.log
    along with the current event count, so we can correlate memory
    growth with event-recording cycles across respawns. Also triggers a
    preemptive graceful restart when VmRSS exceeds a safety threshold:
    jemalloc keeps idle leak near zero but event-time churn still creeps
    ~75 KB/min HWM, so without this the process would eventually OOM
    despite Phase 1 + jemalloc. Graceful restart releases CVI resources
    cleanly (no VPSS/ION stuck state) and avoids the reboot path."""
    log_path = '/tmp/stream_yolo_rss.log'
    events_dir = '/root/ninti_events'
    # Trigger restart at 30 MB RSS; OOM kicks in around 50-60 MB on this
    # 128 MB device. Also restart if VmData crosses 45 MB: the remaining
    # growth is mostly virtual mappings, but bounding it avoids finding out
    # later that a retained extent can become resident under pressure.
    # Cool-off prevents restart loops if jemalloc has a one-time high-water
    # mark that resists shrinking.
    RSS_RESTART_KB = 30_000
    DATA_RESTART_KB = 45_000
    RSS_RESTART_COOL_S = 30 * 60
    last_pid = None
    last_restart_ts = 0
    while True:
        time.sleep(30)
        with yolo_lock:
            proc = yolo_proc
            pid = proc.pid if proc and proc.poll() is None else None
        if pid is None:
            continue
        stats = {}
        try:
            with open(f'/proc/{pid}/status') as f:
                for line in f:
                    if line.startswith(('VmRSS:', 'VmSize:', 'VmData:', 'VmPeak:')):
                        k, v = line.split(':', 1)
                        stats[k.strip()] = v.strip().split()[0]
        except Exception:
            continue
        try:
            events = sum(1 for e in os.scandir(events_dir)
                         if e.name.startswith('event_'))
        except Exception:
            events = -1
        ts = time.strftime('%Y-%m-%d %H:%M:%S')
        try:
            with open(log_path, 'a') as f:
                if last_pid is not None and pid != last_pid:
                    f.write(f'{ts} ---- pid changed {last_pid} -> {pid} (respawn) ----\n')
                f.write(f'{ts} pid={pid} VmRSS={stats.get("VmRSS","?")}kB '
                        f'VmSize={stats.get("VmSize","?")}kB '
                        f'VmData={stats.get("VmData","?")}kB '
                        f'events={events}\n')
        except Exception:
            pass
        last_pid = pid
        try:
            rss_kb = int(stats.get('VmRSS', '0'))
        except Exception:
            rss_kb = 0
        try:
            data_kb = int(stats.get('VmData', '0'))
        except Exception:
            data_kb = 0
        now_ts = time.time()
        restart_reason = None
        if rss_kb > RSS_RESTART_KB:
            restart_reason = f'VmRSS={rss_kb}kB threshold={RSS_RESTART_KB}kB'
        elif data_kb > DATA_RESTART_KB:
            restart_reason = f'VmData={data_kb}kB threshold={DATA_RESTART_KB}kB'
        if restart_reason and now_ts - last_restart_ts > RSS_RESTART_COOL_S:
            last_restart_ts = now_ts
            try:
                with open(log_path, 'a') as f:
                    f.write(f'{ts} ---- soft restart: {restart_reason} ----\n')
            except Exception:
                pass
            print(f'[rss] {restart_reason}, soft restarting stream_yolo')
            stop_yolo()
            start_yolo()

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
        # Only connect MJPEG when an external client is actually watching the SSE
        # stream - otherwise the connection forces stream_yolo to encode JPEG
        # every frame (~400ms on cv181x). Idle: poll once a second.
        if not _stream_interest():
            prev_gray = None
            time.sleep(1)
            continue
        try:
            conn = _hc.HTTPConnection('127.0.0.1', MJPEG_PORT, timeout=5)
            conn.request('GET', '/')
            resp = conn.getresponse()
            buf = b''
            while True:
                if not _stream_interest():
                    try: conn.close()
                    except: pass
                    break
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
                        now_t = time.time()
                        _publish_stream_frame(jpg)
                        _push_pre_buf(jpg, now_t)
                        _record_event_frame(jpg, now_t)
                        with config_lock:
                            motion_on  = cfg.get('motion_enabled', True)
                            sensitivity = cfg.get('motion_sensitivity', MOTION_SENSITIVITY)
                        if not motion_on:
                            prev_gray = None
                            continue
                        # Rate-limit: skip frames to avoid blocking the MJPEG write queue
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
            if '_status' in det:
                _ws_state.update({k: v for k, v in det.items() if k != '_status'})
                continue
            if '_fps' in det or '_active_zone' in det:
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
            _ws_broadcast({'type': 'det', **det})
        except Exception:
            pass

# ---- HTML served from index.html (read from disk each request) ----
_INDEX_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'index.html')
_HA_STREAM_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ha-stream.html')

def _read_index_html():
    try:
        with open(_INDEX_PATH, 'rb') as f:
            return f.read()
    except Exception:
        return b'<h1>index.html not found</h1>'

def _read_ha_stream_html():
    try:
        with open(_HA_STREAM_PATH, 'rb') as f:
            return f.read()
    except Exception:
        return b'<h1>ha-stream.html not found</h1>'

if False:
    HTML = b'''<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NintiDetect</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#111;color:#eee;display:flex;flex-direction:column;height:100vh;overflow:hidden}

/* top nav */
#topnav{display:flex;align-items:center;height:46px;background:#141414;border-bottom:1px solid #2a2a2a;padding:0 16px;flex:0 0 auto;gap:0}
#nav-brand{font-size:12px;letter-spacing:.12em;color:#8cf;font-weight:bold;margin-right:28px;user-select:none}
#nav-tabs{display:flex;height:100%}
.nav-tab{padding:0 22px;height:100%;background:transparent;border:0;border-bottom:3px solid transparent;font-family:monospace;font-size:11px;letter-spacing:.08em;color:#555;cursor:pointer;text-transform:uppercase;transition:color .12s;white-space:nowrap}
.nav-tab:hover{color:#aaa}
.nav-tab.active{color:#8cf;border-bottom-color:#8cf}
#nav-status{margin-left:auto;font-size:11px;color:#555;display:flex;align-items:center;gap:10px}
#status{color:#8cf;font-size:11px}

/* main tab pages */
.main-tab{display:none;flex:1;min-height:0;overflow:hidden}
.main-tab.active{display:flex}

/* live tab */
#zone-panel{width:200px;flex:0 0 200px;padding:10px;background:#141414;display:flex;flex-direction:column;border-left:1px solid #2a2a2a;overflow-y:auto;font-size:12px;gap:10px}
#zone-panel .panel-head{font-size:10px;color:#555;border-bottom:1px solid #222;padding-bottom:4px;margin-bottom:6px;letter-spacing:.1em;text-transform:uppercase}
.zp-section{display:flex;flex-direction:column;gap:4px}
.zp-section h3{font-size:10px;color:#555;letter-spacing:.08em;text-transform:uppercase;margin-bottom:2px}
#pipeline-panel{width:176px;flex:0 0 176px;padding:12px 10px;background:#141414;display:flex;flex-direction:column;border-right:1px solid #2a2a2a;overflow-y:auto;font-size:12px}
.panel-head{font-size:10px;color:#555;border-bottom:1px solid #222;padding-bottom:4px;margin-bottom:10px;letter-spacing:.1em;text-transform:uppercase}
#stream-area{flex:1;position:relative;display:flex;align-items:center;justify-content:center;background:#000;overflow:hidden}
#stream{display:block;width:100%;height:100%;object-fit:contain;image-rendering:pixelated}
#hlsPlayer{display:none;width:100%;height:100%;object-fit:contain}
#hlsCanvas{display:none;width:100%;height:100%}
#overlay{position:absolute;cursor:crosshair}
#stream-btns{position:absolute;bottom:10px;left:50%;transform:translateX(-50%);display:flex;gap:6px}

/* pipeline */
.pipeline{display:flex;flex-direction:column;align-items:stretch;gap:0}
.pnode{border:1px solid #2a2a2a;padding:7px 10px;background:#1a1a1a;transition:background .15s,border-color .15s;cursor:default}
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
.pstat{margin-top:14px;border-top:1px solid #222;padding-top:10px;display:flex;flex-direction:column;gap:6px}
.st-row{display:flex;flex-direction:column;gap:1px}
.st-label{font-size:10px;color:#555}
.st-val{font-size:13px;color:#ccc}
.st-val.on{color:#0f0}
.st-val.off{color:#555}
.st-val.warn{color:#fa5}

/* events tab */
#main-events{overflow-y:auto;display:none}
#main-events.active{display:block}
#ev-toolbar{position:sticky;top:0;z-index:2;background:#0e0e0e;border-bottom:1px solid #2a2a2a;padding:8px 16px;display:flex;align-items:center;gap:10px}
.ev-title{font-size:10px;color:#555;letter-spacing:.1em;text-transform:uppercase;flex:1}
.ev-count{font-size:11px;color:#555}
#eventsLeft{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:2px;padding:2px;background:#0a0a0a}
.ev-mini{display:flex;flex-direction:column;cursor:pointer;overflow:hidden;background:#141414;transition:opacity .15s}
.ev-mini:hover{opacity:.8}
.ev-mini .ev-thumb-wrap{aspect-ratio:16/9;overflow:hidden;background:#000}
.ev-mini .ev-thumb-wrap img{width:100%;height:100%;object-fit:cover;display:block}
.ev-mini .ev-thumb-wrap .ev-no-thumb{width:100%;height:100%;display:flex;align-items:center;justify-content:center;font-size:22px;color:#333}
.ev-mini .ev-info{padding:8px 10px;flex:0 0 auto}
.ev-mini .ev-head{display:flex;align-items:center;gap:6px;margin-bottom:3px}
.ev-mini .ev-cat{font-size:13px;color:#8cf;font-weight:bold;text-transform:uppercase;letter-spacing:.05em;flex:1;overflow:hidden;white-space:nowrap;text-overflow:ellipsis}
.ev-mini .ev-score{font-size:13px;color:#8f8;flex:0 0 auto}
.ev-mini .muted{font-size:12px;color:#777;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.ev-mini .ev-dur{font-size:12px;color:#555}
.ev-mini .ev-meta-row{display:flex;align-items:center;gap:6px;margin-top:3px}
.ev-mini .ev-id{font-size:11px;color:#555;flex:0 0 auto}
.ev-mini .ev-dt{font-size:11px;color:#555;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.thumb{display:none}
.ev-noresult{grid-column:1/-1;padding:40px 20px;color:#555;font-size:12px;text-align:center}
/* lightbox */
#ev-lightbox{display:none;position:fixed;inset:0;z-index:100;background:rgba(0,0,0,.92);flex-direction:column;align-items:center;justify-content:center}
#ev-lightbox.open{display:flex}
#ev-lb-close{position:absolute;top:14px;right:18px;font-size:22px;color:#888;cursor:pointer;line-height:1;background:none;border:none;padding:4px 8px}
#ev-lb-close:hover{color:#fff}
#ev-lb-inner{position:relative;max-width:90vw;max-height:82vh;display:flex;align-items:center;justify-content:center}
#eventPlayer{max-width:90vw;max-height:78vh;background:#000;display:none}
#eventFramePlayer{max-width:90vw;max-height:78vh;object-fit:contain;background:#000;display:none}
#ev-meta{margin-top:10px;font-size:11px;color:#666;text-align:center;line-height:1.8}

/* detection-zone preview in settings */
/* settings tab */
#main-settings{overflow-y:auto;display:none}
#main-settings.active{display:block}
#settings-inner{max-width:560px;margin:0 auto;padding:20px 16px;display:flex;flex-direction:column;gap:14px}
.s-section{background:#141414;border:1px solid #2a2a2a;padding:14px 16px}
.s-section h2{font-size:10px;color:#666;border-bottom:1px solid #222;padding-bottom:5px;margin-bottom:10px;letter-spacing:.1em;text-transform:uppercase}
.s-row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:8px}
.s-row:last-child{margin-bottom:0}
.s-label{font-size:11px;color:#666;min-width:80px}

/* common */
input[type=range]{width:100%}
input[type=text],select{background:#222;color:#eee;border:1px solid #555;padding:4px 6px;font-family:monospace;font-size:12px}
button{background:#2a2a2a;color:#ddd;border:1px solid #555;padding:5px 10px;cursor:pointer;font-family:monospace;font-size:12px}
button:hover{background:#3a3a3a}
button.red{border-color:#633;color:#f88}
button.red:hover{background:#2a1010}
button.act{background:#1a3a1a;border-color:#4a8a4a;color:#8f8}
.row{display:flex;gap:6px;align-items:center;flex-wrap:wrap}
#log{height:200px;overflow-y:auto;background:#0a0a0a;padding:6px;border:1px solid #2a2a2a;line-height:1.5}
.ll{color:#0f0;padding:1px 0;font-size:12px}
.hint{color:#555;font-size:11px}
.zi{background:#1e1e1e;border:1px solid #333;padding:5px;margin-bottom:4px}
.zn{font-weight:bold}
.mode-radio{display:flex;gap:10px;flex-wrap:wrap}
.mode-radio label{display:flex;align-items:center;gap:4px;cursor:pointer;padding:5px 10px;border:1px solid #444}
.mode-radio label.active{border-color:#4a8a4a;background:#1a3a1a;color:#8f8}
.dot{width:7px;height:7px;border-radius:50%;background:#555;display:inline-block}
.dot.on{background:#0f0}
.dot.off{background:#555}
</style>
</head>
<body>

<div id="topnav">
  <span id="nav-brand">NINTIDETECT</span>
  <div id="nav-tabs">
    <button class="nav-tab active" onclick="showMainTab('live',this)">&#9654; Live</button>
    <button class="nav-tab" onclick="showMainTab('events',this)">Events</button>
    <button class="nav-tab" onclick="showMainTab('settings',this)">Settings</button>
  </div>
  <div id="nav-status">
    <span class="dot" id="dotYolo"></span><span id="stMode" style="margin-left:5px">-</span>
    <span id="stTemp">-</span>
    <span id="status"></span>
  </div>
</div>

<!-- -- Tab 1: Live -- -->
<div class="main-tab active" id="main-live">

  <div id="pipeline-panel">
    <div class="panel-head">Pipeline</div>
    <div class="pipeline">
      <div class="pnode" id="pn-motion">
        <div class="pname">MOTION</div>
        <div class="pinfo" id="pi-motion">-</div>
      </div>
      <div class="parrow" id="pa-motion">&#x25BC;</div>
      <div class="pnode" id="pn-ai">
        <div class="pname">AI DETECT</div>
        <div class="pinfo" id="pi-ai">-</div>
      </div>
      <div class="parrow" id="pa-ai">&#x25BC;</div>
      <div class="pnode" id="pn-track">
        <div class="pname">TRACKING</div>
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
      <div class="st-row"><span class="st-label">detector</span><span class="st-val off" id="stYolo">-</span></div>
      <div class="st-row"><span class="st-label">tracks</span><span class="st-val" id="stTracks">0</span></div>
      <div class="st-row"><span class="st-label">threshold</span><span class="st-val" id="stThresh">-</span></div>
      <div class="st-row"><span class="st-label">fps</span><span class="st-val" id="stFps">-</span></div>
      <div class="st-row"><span class="st-label">last det</span><span class="st-val" id="stLastDet" style="font-size:10px;line-height:1.4">-</span></div>
    </div>
  </div>

  <div id="stream-area">
    <img id="stream" src="" alt="stream offline">
    <video id="hlsPlayer" muted autoplay playsinline style="display:none"></video>
    <canvas id="hlsCanvas" style="display:none;width:100%;height:100%;object-fit:contain"></canvas>
    <canvas id="overlay"></canvas>
    <div id="stream-btns">
      <button onclick="setStream('hls')"   id="btnHls">HLS</button>
      <button onclick="setStream('mjpeg')" id="btnMjpeg">MJPEG</button>
    </div>
  </div>

  <div id="zone-panel">
    <div class="zp-section">
      <div class="panel-head">Detection Zones</div>
      <button onclick="addDetZone()" style="width:100%">+ Add zone</button>
      <button onclick="delLastDetZone()" class="red" style="width:100%">- Remove last</button>
      <button id="activeBtn" onclick="toggleActiveDetector()" style="width:100%">Zone 1 follower: OFF</button>
      <div class="hint" style="font-size:10px;color:#555;margin-top:2px">Follows detected target when ON</div>
      <div id="dzlist" class="hint"></div>
      <button onclick="applyAll()" class="act apply-all-btn" style="width:100%">Apply</button>
    </div>
    <div class="zp-section">
      <div class="panel-head">Filter Zones</div>
      <div style="display:flex;gap:4px">
        <input id="zname" type="text" placeholder="zone name" style="flex:1;min-width:0">
        <button onclick="newZone()">+</button>
        <button onclick="cancelZone()" class="red">x</button>
      </div>
      <div id="zlist"></div>
    </div>
  </div>

</div>

<!-- -- Tab 2: Events -- -->
<div class="main-tab" id="main-events">
  <div id="ev-toolbar">
    <span class="ev-title">Events</span>
    <span class="ev-count" id="eventStatusLeft"></span>
    <button onclick="loadEvents()" style="font-size:10px;padding:2px 8px">Refresh</button>
  </div>
  <div id="eventsLeft"><div class="ev-noresult">No events yet</div></div>
</div>

<!-- lightbox -->
<div id="ev-lightbox">
  <button id="ev-lb-close" onclick="closeLightbox()">x</button>
  <div id="ev-lb-inner">
    <video id="eventPlayer" controls muted playsinline></video>
    <img id="eventFramePlayer" alt="">
  </div>
  <div id="ev-meta"></div>
</div>

<!-- -- Tab 3: Settings -- -->
<div class="main-tab" id="main-settings">
<div id="settings-inner">

  <div class="s-section">
    <h2>Connection</h2>
    <div class="s-row">
      <input id="ip" type="text" placeholder="board IP" style="flex:1" value="">
      <button onclick="connect()">Connect</button>
    </div>
  </div>

  <div class="s-section">
    <h2>Mode</h2>
    <div class="mode-radio" id="modeRadio">
      <label id="lbl_ninti"><input type="radio" name="mode" value="ninti" onchange="pendingMode='ninti';updateModeLabels()"> NintiDetect</label>
      <label id="lbl_rtsp"><input type="radio" name="mode" value="rtsp" onchange="pendingMode='rtsp';updateModeLabels()"> RTSP</label>
    </div>
    <div class="s-row" style="margin-top:8px">
      <button onclick="applyMode()" style="flex:1">Switch Mode</button>
      <span class="hint" id="modeStatus"></span>
    </div>
    <div class="row hint" style="margin-top:4px;gap:8px">
      <span class="dot" id="dotRtsp"></span> rtsp
    </div>
  </div>

  <div class="s-section">
    <h2>Camera</h2>
    <div class="s-row">
      <span class="s-label">Resolution</span>
      <select id="senRes" style="flex:1">
        <option value="1280,720">1280 x 720</option>
        <option value="1920,1080">1920 x 1080 (HD)</option>
        <option value="2560,1440">2560 x 1440 (QHD)</option>
      </select>
      <button onclick="rebootDevice()" style="margin-left:6px">Reboot</button>
    </div>
    <div class="s-row">
      <span class="s-label">FPS cap</span>
      <input type="range" id="fpsCap" min="0" max="30" step="1" value="15" style="flex:1"
             oninput="document.getElementById('fpsv').textContent=this.value==0?'off':this.value">
      <span id="fpsv" style="width:32px;text-align:right">15</span>
    </div>
  </div>

  <div class="s-section">
    <h2>Threshold</h2>
    <div class="s-row">
      <input type="range" id="th" min="0.05" max="0.95" step="0.05" value="0.45" style="flex:1"
             oninput="document.getElementById('thv').textContent=parseFloat(this.value).toFixed(2)">
      <span id="thv" style="width:36px;text-align:right">0.45</span>
    </div>
  </div>

  <div class="s-section">
    <h2>Motion Detect</h2>
    <div class="s-row">
      <button id="btnMotion" onclick="toggleMotion()" style="flex:1">-</button>
    </div>
    <div class="s-row">
      <span class="s-label">Sensitivity</span>
      <input type="range" id="mth" min="5" max="50" step="5" value="20" style="flex:1"
             oninput="document.getElementById('mthv').textContent=this.value">
      <span id="mthv" style="width:28px;text-align:right">20</span>
    </div>
  </div>

  <div class="s-section">
    <h2>Filter Classes</h2>
    <div id="fclasses"></div>
  </div>

  <div class="s-section">
    <h2>Detections Log</h2>
    <div id="log"></div>
  </div>

  <button onclick="applyAll()" class="act apply-all-btn" style="width:100%;padding:10px;font-size:13px;margin-top:8px">Apply</button>

</div>
</div>

<!-- hidden compat elements -->
<div style="display:none">
  <div id="events"></div>
  <span id="eventStatus"></span>
  <span id="stRtsp"></span>
</div>

<script src="/hls.min.js"></script>
<script>
const PALETTE=['#f55','#5af','#5f5','#fa5','#a5f','#ff5','#5ff'];
const INFER_SZ=640; const MAX_DET_ZONES=4;
let liveActiveZone=null;  // [x,y,size] from stream_yolo when active follower is on
let liveActiveZoneTs=0;
// When the user drags the active zone, suppress live-position overrides on the
// render so they can see their new size/position. Cleared by applyDetector
// (i.e. once the new value has been sent to stream_yolo).
let dzCfgDirty=false;
let cfg={mode:'ninti',threshold:0.45,zones:[],filter_classes:['person','vehicle','animal'],
         detection_zones:[],active_detector:false,sensor_width:1280,sensor_height:720};
let drawing=null;
let hover=null;
let flashes=[];
let ip='';
let es=null;
let pendingMode='ninti';
let dzDrag=null;  // {idx,offX,offY} during detection-zone drag
let eventPlaybackTimer=null;
var _hlsObj=null;
var _streamMode='mjpeg';

const streamEl=document.getElementById('stream');
const cvs=document.getElementById('overlay');
const ctx=cvs.getContext('2d');

if(location.hostname && location.hostname!=='localhost'){
  document.getElementById('ip').value=location.hostname;
}

function getip(){return document.getElementById('ip').value.trim()||location.hostname;}

function connect(){
  ip=getip();
  loadConf();
  loadEvents();
  openSSE();
  pollStatus();
  setInterval(loadEvents, 5000);
  setStream('hls');
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
      if(!Array.isArray(cfg.detection_zones)) cfg.detection_zones=[];
      cfg.detection_zones.forEach(z=>{ if(!z.size) z.size=INFER_SZ; });
      cfg.active_detector = !!c.active_detector;
      cfg.sensor_width = c.sensor_width||1280;
      cfg.sensor_height= c.sensor_height||720;
      const sr=document.getElementById('senRes');
      if(sr) sr.value=cfg.sensor_width+','+cfg.sensor_height;
      const fc=document.getElementById('fpsCap');
      const tfps=(c.target_fps==null)?15:c.target_fps;
      if(fc){fc.value=tfps; document.getElementById('fpsv').textContent=tfps==0?'off':tfps;}
      renderDzList();
      renderZones(); renderClasses();
    }).catch(e=>setStatus('load config failed: '+e));
}

function saveConf(){
  fetch('http://'+ip+':7778/api/config',{
    method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(cfg)
  }).then(r=>r.json()).then(()=>setStatus('Saved.')).catch(e=>setStatus('save failed: '+e));
}

function loadEvents(){
  const listEl=document.getElementById('eventsLeft');
  const countEl=document.getElementById('eventStatusLeft');
  if(!ip)return;
  fetch('http://'+ip+':7778/api/events')
    .then(r=>r.json()).then(rows=>{
      if(countEl) countEl.textContent=rows.length+' events';
      if(!listEl) return;
      if(!rows.length){
        listEl.innerHTML='<div class="ev-noresult">No events yet</div>';
        return;
      }
      listEl.innerHTML='';
      rows.slice().reverse().forEach(e=>{
        const hasVideo=!!(e.video_url||e.frames_url||e.event_meta_url);
        const dur=e.duration_s!==undefined ? Number(e.duration_s).toFixed(1)+'s' : '';
        const sc=e.best_score!==undefined ? (Number(e.best_score)*100).toFixed(0)+'%' : '';
        const thumb=thumbForEvent(e);
        const item=document.createElement('div');
        item.className='ev-mini'+(hasVideo?' video':'');
        const ts=e.first_seen||e.t;
        const dt=ts?new Date(ts*1000).toLocaleString('en-GB',{day:'2-digit',month:'short',hour:'2-digit',minute:'2-digit',hour12:false}):'';
        const eid=e.track_id!=null?'#'+e.track_id:'';
        item.innerHTML=
          `<div class="ev-thumb-wrap">`+
            (thumb?`<img src="http://${ip}:7778${thumb}" loading="lazy">`:'<div class="ev-no-thumb">?</div>')+
          `</div>`+
          `<div class="ev-info">`+
            `<div class="ev-head">`+
              `<span class="ev-cat">${e.cat||'?'}</span>`+
              `<span class="ev-score">${sc}</span>`+
            `</div>`+
            `<div class="muted">${e.name||''}&nbsp;<span class="ev-dur">${dur}</span></div>`+
            `<div class="ev-meta-row"><span class="ev-id">${eid}</span><span class="ev-dt">${dt}</span></div>`+
          `</div>`;
        if(hasVideo){
          item.onclick=()=>openLightbox(e);
        }
        listEl.appendChild(item);
      });
      listEl.scrollTop=0;
    }).catch(()=>{
      if(countEl) countEl.textContent='failed';
    });
}

function openLightbox(e){
  const lb=document.getElementById('ev-lightbox');
  lb.classList.add('open');
  document.getElementById('ev-meta').textContent='';
  playEvent(e);
}
function closeLightbox(){
  const lb=document.getElementById('ev-lightbox');
  lb.classList.remove('open');
  const player=document.getElementById('eventPlayer');
  player.pause();
  player.src='';
  player.style.display='none';
  if(eventPlaybackTimer){clearInterval(eventPlaybackTimer);eventPlaybackTimer=null;}
  document.getElementById('eventFramePlayer').style.display='none';
}
document.addEventListener('keydown',e=>{if(e.key==='Escape')closeLightbox();});
document.getElementById('ev-lightbox').addEventListener('click',function(e){if(e.target===this)closeLightbox();});
function playEvent(e){
  if(!e)return;
  const metaUrl=e.event_meta_url || (e.event_dir ? e.event_dir+'/event.json' : (e.video_url ? e.video_url.replace(/event\\.mp4$/, 'event.json') : ''));
  if(metaUrl){
    fetch('http://'+ip+':7778'+metaUrl)
      .then(r=>r.json()).then(meta=>playEventFrames({...e,...meta}))
      .catch(()=>playEventFrames(e));
  } else {
    playEventFrames(e);
  }
}
function playEventFrames(e){
  if(eventPlaybackTimer){clearInterval(eventPlaybackTimer);eventPlaybackTimer=null;}
  const player=document.getElementById('eventPlayer');
  player.pause();
  player.style.display='none';
  // Prefer mp4 when available; JPEG slideshow is fallback only
  if(e.video_url){playEventMp4(e.video_url);return;}
  const framePlayer=document.getElementById('eventFramePlayer');
  const framesUrl=e.frames_url || (e.event_dir ? e.event_dir+'/frames/' : '');
  const n=parseInt(e.num_frames||0);
  const fps=parseFloat(e.record_fps||25);
  if(!framesUrl || !n){
    return;
  }
  let idx=0;
  framePlayer.style.display='block';
  const urls=[];
  for(let i=0;i<n;i++){
    urls.push('http://'+ip+':7778'+framesUrl+'frame_'+String(i).padStart(5,'0')+'.jpg');
  }
  const cache=urls.map(u=>{const img=new Image(); img.src=u; return img;});
  const step=()=>{
    framePlayer.src=cache[idx].src;
    idx=(idx+1)%n;
  };
  step();
  eventPlaybackTimer=setInterval(step, Math.max(100, 1000/fps));
}
function playEventMp4(url){
  if(!url)return;
  if(eventPlaybackTimer){clearInterval(eventPlaybackTimer);eventPlaybackTimer=null;}
  document.getElementById('eventFramePlayer').style.display='none';
  const player=document.getElementById('eventPlayer');
  player.style.display='block';
  player.src='http://'+ip+':7778'+url;
  player.play().catch(()=>{});
}
function thumbForEvent(e){
  if(e.thumbnail_url)return e.thumbnail_url;
  if(e.video_url)return e.video_url.replace(/event\\.mp4$/, 'best_frame.jpg');
  return '';
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
        if(stR){stR.textContent=s.rtsp?'running':'stopped';
          stR.className='st-val '+(s.rtsp?'on':'off');}
        const trackCount=s.tracks||0;
        document.getElementById('stTracks').textContent=trackCount;
        document.getElementById('stThresh').textContent=parseFloat(cfg.threshold||0).toFixed(2);
        const stT=document.getElementById('stTemp');
        if(s.soc_temp!=null){stT.textContent=s.soc_temp.toFixed(1)+'\\u00b0C';
          stT.className='st-val '+(s.soc_temp>=85?'warn':s.soc_temp>=75?'warn':'on');}
        else{stT.textContent='-';stT.className='st-val';}
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
  es.onmessage=e=>{try{const d=JSON.parse(e.data);if(d._fps!==undefined)onFps(d);else if(d._active_zone)onActiveZone(d);else if(d._motion)onMotion(d);else if(d._save)onSave(d);else if(d._discard)onDiscard(d);else onDet(d);}catch{}};
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
function onActiveZone(d){
  liveActiveZone=d._active_zone;
  liveActiveZoneTs=Date.now();
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
  const hlsEl=document.getElementById('hlsPlayer');
  const activeEl=(_streamMode==='hls'&&hlsEl.videoWidth)?hlsEl:streamEl;
  const lr=activeEl.getBoundingClientRect();
  const pr=activeEl.parentElement.getBoundingClientRect();
  const nw=(activeEl===hlsEl?hlsEl.videoWidth:activeEl.naturalWidth)||640;
  const nh=(activeEl===hlsEl?hlsEl.videoHeight:activeEl.naturalHeight)||360;
  // compute actual rendered rect (object-fit:contain inside lr)
  const scale=Math.min(lr.width/nw, lr.height/nh);
  const rw=nw*scale, rh=nh*scale;
  const rx=(lr.width-rw)/2, ry=(lr.height-rh)/2;
  cvs.width=nw;
  cvs.height=nh;
  cvs.style.left=(lr.left-pr.left+rx)+'px';
  cvs.style.top=(lr.top-pr.top+ry)+'px';
  cvs.style.width=rw+'px';
  cvs.style.height=rh+'px';
}
streamEl.onload=syncCvs;
document.getElementById('hlsPlayer').addEventListener('loadedmetadata',syncCvs);
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

// Detection-zone drag (yellow boxes) -- takes precedence over polygon drawing.
function eventToPx(e){
  const [nx,ny]=canvasToNorm(e);
  return [nx*cfg.sensor_width, ny*cfg.sensor_height];
}
function eventToPxOn(e, theCanvas){
  const r=theCanvas.getBoundingClientRect();
  const vx=(e.clientX-r.left)/r.width, vy=(e.clientY-r.top)/r.height;
  const [nx,ny]=viewToRaw(vx,vy);
  return [nx*cfg.sensor_width, ny*cfg.sensor_height];
}
function zoneSize(z){ return z.size||INFER_SZ; }
// 32 sensor-px handle in the bottom-right corner; returns 'resize' or 'move'.
function hitDzAt(px,py){
  const zs=cfg.detection_zones||[];
  for(let i=zs.length-1;i>=0;i--){
    // For the active follower (zone 0), hit-test against the visible live
    // position rather than the static config position, otherwise the user
    // clicks the orange box and nothing happens.
    let zx=zs[i].x, zy=zs[i].y, s=zoneSize(zs[i]);
    if(i===0 && cfg.active_detector && liveActiveZone && !dzCfgDirty){
      zx=liveActiveZone[0]; zy=liveActiveZone[1]; s=liveActiveZone[2];
    }
    if(px>=zx && px<zx+s && py>=zy && py<zy+s){
      const inHandle = px>=zx+s-32 && py>=zy+s-32;
      return {idx:i, mode: inHandle?'resize':'move'};
    }
  }
  return null;
}
cvs.addEventListener('mousedown',e=>{
  if(drawing) return;
  const [px,py]=eventToPx(e);
  const hit=hitDzAt(px,py);
  if(!hit) return;
  const z=cfg.detection_zones[hit.idx];
  // Active zone hit: snapshot only the live tracked X/Y into config so the
  // drag operates on the visible position. Size is preserved -- if the user
  // already resized but hasn't applied, that pending size stays. Suppress
  // live overrides until apply.
  if(hit.idx===0 && cfg.active_detector && liveActiveZone){
    z.x=liveActiveZone[0]; z.y=liveActiveZone[1];
    dzCfgDirty=true;
  }
  if(hit.mode==='resize'){
    dzDrag={idx:hit.idx, mode:'resize', dragged:false, srcCanvas:cvs};
  } else {
    dzDrag={idx:hit.idx, mode:'move', offX:px-z.x, offY:py-z.y, dragged:false, srcCanvas:cvs};
  }
  e.preventDefault();
});
window.addEventListener('mousemove',e=>{
  if(!dzDrag) return;
  const [px,py]=eventToPxOn(e, dzDrag.srcCanvas||cvs);
  const sw=cfg.sensor_width, sh=cfg.sensor_height;
  const z=cfg.detection_zones[dzDrag.idx];
  if(dzDrag.mode==='resize'){
    let ns=Math.round(Math.min(px-z.x, py-z.y));
    ns=Math.max(INFER_SZ, ns);  // minimum size only; may exceed frame
    z.size=ns;
  } else {
    let nx=Math.round(px-dzDrag.offX), ny=Math.round(py-dzDrag.offY);
    z.x=nx; z.y=ny;  // no frame-bound clamping; zone may extend beyond image
  }
  dzDrag.dragged=true;
  renderDzList();
});
window.addEventListener('mouseup',()=>{ if(dzDrag) dzDrag=null; });

cvs.addEventListener('mousemove',e=>{if(drawing)hover=canvasToNorm(e);});
cvs.addEventListener('click',e=>{
  if(dzDrag) return;  // drag took precedence (mouseup may race)
  if(!drawing){
    // Suppress click if it ended a drag
    const [px,py]=eventToPx(e);
    if(hitDzAt(px,py)) return;
    return;
  }
  const pt=canvasToNorm(e);
  if(drawing.pts.length>=3&&dist(pt,drawing.pts[0])<0.04){closeZone();return;}
  drawing.pts.push(pt);
});
cvs.addEventListener('contextmenu',e=>{e.preventDefault();closeZone();});

// Detection-zone CRUD + apply
function addDetZone(){
  if(!cfg.detection_zones) cfg.detection_zones=[];
  if(cfg.detection_zones.length>=MAX_DET_ZONES){setStatus('Max '+MAX_DET_ZONES+' zones.');return;}
  const sw=cfg.sensor_width, sh=cfg.sensor_height;
  const cx=Math.round((sw-INFER_SZ)/2), cy=Math.round((sh-INFER_SZ)/2);
  const off=cfg.detection_zones.length*40;
  cfg.detection_zones.push({
    x:Math.max(0,Math.min(cx+off,sw-INFER_SZ)),
    y:Math.max(0,Math.min(cy+off,sh-INFER_SZ)),
    size:INFER_SZ});
  renderDzList();
}
function delLastDetZone(){
  if(!cfg.detection_zones||!cfg.detection_zones.length)return;
  cfg.detection_zones.pop(); renderDzList();
}
function toggleActiveDetector(){
  if(cfg.active_detector && liveActiveZone && cfg.detection_zones && cfg.detection_zones[0]){
    cfg.detection_zones[0].x=liveActiveZone[0];
    cfg.detection_zones[0].y=liveActiveZone[1];
  }
  cfg.active_detector=!cfg.active_detector;
  if(!cfg.active_detector) liveActiveZone=null;
  const b=document.getElementById('activeBtn');
  if(b) b.textContent='Zone 1 follower: '+(cfg.active_detector?'ON':'OFF');
  renderDzList();
  // apply immediately -- no separate button press needed
  fetch('http://'+ip+':7778/api/config',{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({active_detector:!!cfg.active_detector,_restart:true})
  }).then(()=>setStatus('Follower '+(cfg.active_detector?'ON':'OFF')+', restarting...'))
    .catch(e=>setStatus(''+e));
}
function renderDzList(){
  const el=document.getElementById('dzlist');
  if(!el)return;
  const zs=cfg.detection_zones||[];
  if(!zs.length){el.textContent='No zones -- stream_yolo defaults to single center zone.';return;}
  el.innerHTML=zs.map((z,i)=>{
    const tag=(i===0 && cfg.active_detector)?' <span style="color:#ff8800">[active]</span>':'';
    return `<div>#${i+1} at (${z.x},${z.y}) size=${z.size||INFER_SZ}${tag}</div>`;
  }).join('');
  const b=document.getElementById('activeBtn');
  if(b) b.textContent='Zone 1 follower: '+(cfg.active_detector?'ON':'OFF');
}
function applyAll(){
  const sr=document.getElementById('senRes').value.split(',');
  cfg.sensor_width=parseInt(sr[0]);
  cfg.sensor_height=parseInt(sr[1]);
  const sw=cfg.sensor_width, sh=cfg.sensor_height;
  (cfg.detection_zones||[]).forEach(z=>{
    z.size=Math.max(INFER_SZ,z.size||INFER_SZ);  // minimum size only; zone may exceed frame
  });
  cfg.target_fps=parseInt(document.getElementById('fpsCap').value);
  cfg.threshold=parseFloat(document.getElementById('th').value);
  cfg.motion_sensitivity=parseInt(document.getElementById('mth').value);
  renderDzList();
  const btns=document.querySelectorAll('.apply-all-btn');
  btns.forEach(b=>{b.disabled=true;b.textContent='Applying...';});
  fetch('http://'+ip+':7778/api/config',{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(cfg)
  }).then(()=>{
    dzCfgDirty=false;
    setStatus('Saved. Reboot to apply hardware changes.');
    btns.forEach(b=>{b.disabled=false;b.textContent='Apply';});
  }).catch(e=>{
    setStatus(''+e);
    btns.forEach(b=>{b.disabled=false;b.textContent='Apply';});
  });
}
function applyDetector(){applyAll();}

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

  // Detection zones (yellow dashed normal; orange when zone 0 is active follower).
  // While stream_yolo is reporting live _active_zone, zone 0 renders at the
  // tracked position instead of the config position.
  const sw=cfg.sensor_width||1280, sh=cfg.sensor_height||720;
  (cfg.detection_zones||[]).forEach((z,i)=>{
    const isActive=(i===0 && !!cfg.active_detector);
    const dragging=dzDrag&&dzDrag.idx===i;
    let zx=z.x, zy=z.y, zs=z.size||INFER_SZ;
    // Show live position only when active and the user isn't editing.
    if(isActive && liveActiveZone && !dragging && !dzCfgDirty){
      zx=liveActiveZone[0]; zy=liveActiveZone[1]; zs=liveActiveZone[2];
    }
    const x=zx*W/sw, y=zy*H/sh;
    const w=zs*W/sw, h=zs*H/sh;
    const col=isActive?'#ff8800':'#ffcc00';
    ctx.setLineDash([8,4]);
    ctx.strokeStyle=dragging?'#ff0':col;
    ctx.lineWidth=dragging?3:2;
    ctx.strokeRect(x,y,w,h);
    ctx.setLineDash([]);
    ctx.fillStyle=isActive?'rgba(255,136,0,0.10)':'rgba(255,204,0,0.10)';
    ctx.fillRect(x,y,w,h);
    // Resize handle (right-bottom corner) -- 32 sensor px -> scale
    const hsx=Math.max(4, 32*W/sw), hsy=Math.max(4, 32*H/sh);
    ctx.fillStyle=col;
    ctx.fillRect(x+w-hsx, y+h-hsy, hsx, hsy);
    ctx.font='11px monospace';
    ctx.fillText('zone '+(i+1)+(isActive?' [active]':'')+' ('+zx+','+zy+') sz='+zs, x+4, y+13);
  });

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

function rebootDevice(){
  if(!confirm('Reboot device?')) return;
  fetch('http://'+getip()+':7778/api/reboot',{method:'POST'})
    .then(()=>setStatus('Rebooting... (~45s)'))
    .catch(()=>setStatus('Rebooting...'));
}
function setStatus(msg){
  document.getElementById('status').textContent=msg;
  if(msg)setTimeout(()=>{if(document.getElementById('status').textContent===msg)
    document.getElementById('status').textContent='';},4000);
}

function showMainTab(name, btn){
  const wasLive = document.getElementById('main-live').classList.contains('active');
  document.querySelectorAll('.nav-tab').forEach(b=>b.classList.remove('active'));
  document.querySelectorAll('.main-tab').forEach(p=>p.classList.remove('active'));
  btn.classList.add('active');
  document.getElementById('main-'+name).classList.add('active');
  if(name==='events') loadEvents();
  // leaving live tab: tear down HLS so buffer doesn't accumulate
  if(wasLive && name!=='live' && _streamMode==='hls') {
    if(_hlsObj){ _hlsObj.destroy(); _hlsObj=null; }
    const hlsEl=document.getElementById('hlsPlayer');
    hlsEl.pause(); hlsEl.src='';
  }
  // returning to live tab: restart HLS fresh from live edge
  if(!wasLive && name==='live' && _streamMode==='hls') {
    setStream('hls');
  }
}

if(document.getElementById('ip').value) connect();

// ---- HLS player (rotation handled by VPSS hardware: bMirror+bFlip in capture_cvi.cpp) ----
function setStream(mode) {
  const ip      = document.getElementById('ip').value.trim();
  const mjpegEl = document.getElementById('stream');
  const hlsEl   = document.getElementById('hlsPlayer');
  _streamMode = mode;
  if (mode === 'hls') {
    mjpegEl.src = ''; mjpegEl.style.display = 'none';
    hlsEl.style.display = 'block';
    const hlsUrl = 'http://' + ip + ':7778/hls/live.m3u8';
    if (window.Hls && Hls.isSupported()) {
      if (_hlsObj) _hlsObj.destroy();
      _hlsObj = new Hls({
        liveSyncDurationCount: 3,
        liveMaxLatencyDurationCount: 6,
        maxBufferLength: 20,
        maxMaxBufferLength: 20,
        lowLatencyMode: false,
      });
      _hlsObj.loadSource(hlsUrl);
      _hlsObj.attachMedia(hlsEl);
      _hlsObj.on(Hls.Events.MANIFEST_PARSED, () => hlsEl.play().catch(()=>{}));
    } else if (hlsEl.canPlayType('application/vnd.apple.mpegurl')) {
      hlsEl.src = hlsUrl; hlsEl.play().catch(()=>{});
    }
  } else {
    if (_hlsObj) { _hlsObj.destroy(); _hlsObj = null; }
    hlsEl.src = ''; hlsEl.style.display = 'none';
    mjpegEl.style.display = '';
    mjpegEl.src = 'http://' + ip + ':7778/stream';
  }
}
</script>
</body>
</html>'''  # end of dead-code block — actual HTML served from index.html

# ---- HTTP handler ----
class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): pass

    def _cors(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET,POST,OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def _serve_event_file(self, path, head_only=False):
        rel = path[len('/event-video/'):]
        rel = os.path.normpath(rel).lstrip('/')
        full = os.path.abspath(os.path.join(EVENT_DIR, rel))
        root = os.path.abspath(EVENT_DIR)
        if not full.startswith(root + os.sep) or not os.path.exists(full):
            self.send_response(404)
            self.end_headers()
            return
        ctype = 'application/octet-stream'
        if full.endswith('.mp4'):
            ctype = 'video/mp4'
        elif full.endswith('.jpg') or full.endswith('.jpeg'):
            ctype = 'image/jpeg'
        elif full.endswith('.json'):
            ctype = 'application/json'

        size = os.path.getsize(full)
        start, end = 0, size - 1
        status = 200
        rng = self.headers.get('Range', '')
        if rng.startswith('bytes='):
            spec = rng.split('=', 1)[1].split(',', 1)[0].strip()
            a, _, b = spec.partition('-')
            try:
                if a:
                    start = int(a)
                if b:
                    end = int(b)
                if not a and b:
                    start = max(0, size - int(b))
                    end = size - 1
                start = max(0, min(start, size - 1))
                end = max(start, min(end, size - 1))
                status = 206
            except Exception:
                start, end, status = 0, size - 1, 200

        length = max(0, end - start + 1)
        self.send_response(status)
        self.send_header('Content-Type', ctype)
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('Content-Length', str(length))
        if status == 206:
            self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
        self.send_header('Cache-Control', 'no-cache')
        self._cors()
        self.end_headers()
        if head_only:
            return
        with open(full, 'rb') as f:
            f.seek(start)
            remaining = length
            while remaining > 0:
                chunk = f.read(min(1024 * 256, remaining))
                if not chunk:
                    break
                self.wfile.write(chunk)
                remaining -= len(chunk)

    def do_OPTIONS(self):
        self.send_response(200)
        self._cors()
        self.end_headers()

    def do_HEAD(self):
        path = urlparse(self.path).path
        if path.startswith('/event-video/'):
            self._serve_event_file(path, head_only=True)
        elif path in ('/', '/index.html', '/api/status', '/api/config', '/api/events'):
            self.send_response(200)
            self._cors()
            self.end_headers()
        elif path.startswith('/hls/'):
            rel = path[len('/hls/'):]
            rel = os.path.normpath(rel).lstrip('/')
            full = os.path.abspath(os.path.join('/tmp/hls', rel))
            if not full.startswith('/tmp/hls') or not os.path.exists(full):
                self.send_response(404); self.end_headers(); return
            ctype = 'application/vnd.apple.mpegurl' if full.endswith('.m3u8') else 'video/MP2T'
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', os.path.getsize(full))
            self.send_header('Cache-Control', 'no-cache')
            self._cors()
            self.end_headers()
        else:
            self.send_response(200)
            self.end_headers()

    def do_GET(self):
        path = urlparse(self.path).path

        if path in ('/', '/index.html'):
            body = _read_index_html()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif path == '/ha-stream.html':
            body = _read_ha_stream_html()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(body)

        elif path == '/api/config':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            self.wfile.write(json.dumps(public_cfg()).encode())

        elif path == '/api/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            soc_temp = None
            try:
                with open('/sys/class/thermal/thermal_zone0/temp') as _t:
                    soc_temp = round(int(_t.read().strip()) / 1000.0, 1)
            except Exception:
                pass
            status = {
                'mode': cfg.get('mode', 'ninti'),
                'yolo': _yolo_running(),
                'rtsp': _rtsp_running(),
                'tracks': len(tracks),
                'soc_temp': soc_temp,
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
                            rec = json.loads(line)
                            if rec.get('video_url') and not _valid_event_video_url(rec.get('video_url')):
                                continue
                            rows.append(rec)
                        except Exception:
                            pass
            except Exception as e:
                print(f'[events] read error: {e}')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            self.wfile.write(json.dumps(rows).encode())

        elif path.startswith('/event-video/'):
            self._serve_event_file(path)

        elif path == '/hls.min.js':
            hls_js = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hls.min.js')
            if not os.path.exists(hls_js):
                self.send_response(404); self.end_headers(); return
            data = open(hls_js, 'rb').read()
            self.send_response(200)
            self.send_header('Content-Type', 'application/javascript')
            self.send_header('Content-Length', len(data))
            self.send_header('Cache-Control', 'public, max-age=86400')
            self.end_headers()
            self.wfile.write(data)

        elif path.startswith('/hls/'):
            rel = path[len('/hls/'):]
            rel = os.path.normpath(rel).lstrip('/')
            full = os.path.abspath(os.path.join('/tmp/hls', rel))
            if not full.startswith('/tmp/hls') or not os.path.exists(full):
                self.send_response(404); self.end_headers(); return
            ctype = 'application/vnd.apple.mpegurl' if full.endswith('.m3u8') else 'video/MP2T'
            data = open(full, 'rb').read()
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', len(data))
            self.send_header('Cache-Control', 'no-cache')
            self._cors()
            self.end_headers()
            self.wfile.write(data)

        elif path == '/stream':
            global stream_clients
            try:
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-cache')
                self._cors()
                self.end_headers()
                with stream_lock:
                    stream_clients += 1
                    stream_lock.notify_all()
                last_id = -1
                while True:
                    with stream_lock:
                        stream_lock.wait_for(
                            lambda: latest_stream_jpg is not None and latest_stream_id != last_id,
                            timeout=10,
                        )
                        if latest_stream_jpg is None or latest_stream_id == last_id:
                            continue
                        jpg = latest_stream_jpg
                        last_id = latest_stream_id
                    header = (
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        + f'Content-Length: {len(jpg)}\r\n\r\n'.encode()
                    )
                    self.wfile.write(header)
                    self.wfile.write(jpg)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
            except Exception:
                pass
            finally:
                with stream_lock:
                    stream_clients = max(0, stream_clients - 1)

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

        elif path == '/ws':
            self._handle_ws()

        else:
            self.send_response(404)
            self.end_headers()

    def _handle_ws(self):
        key = self.headers.get('Sec-WebSocket-Key', '')
        if not key or self.headers.get('Upgrade', '').lower() != 'websocket':
            self.send_response(400); self.end_headers(); return
        accept = base64.b64encode(
            hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()
        ).decode()
        resp = (
            'HTTP/1.1 101 Switching Protocols\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            f'Sec-WebSocket-Accept: {accept}\r\n'
            '\r\n'
        )
        self.connection.sendall(resp.encode())
        q = Queue()
        with _ws_clients_lock:
            _ws_clients.append(q)
        try:
            while True:
                try:
                    frame = q.get(timeout=30)
                    self.connection.sendall(frame)
                except Empty:
                    self.connection.sendall(bytes([0x89, 0x00]))  # ping
        except Exception:
            pass
        finally:
            with _ws_clients_lock:
                try: _ws_clients.remove(q)
                except ValueError: pass

    def do_POST(self):
        path = urlparse(self.path).path
        n = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(n)

        if path == '/api/config':
            try:
                new = json.loads(body)
                restart = new.pop('_restart', False)
                new.pop('mqtt_has_password', None)
                if new.get('mqtt_password', None) == '':
                    new.pop('mqtt_password', None)
                with config_lock:
                    cfg.update(new)
                save_cfg()
                if any(k.startswith('mqtt_') for k in new):
                    mqtt_state['discovery_sent'] = False
                    mqtt_publish_discovery(force=True)
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

        elif path == '/api/mqtt/test':
            mqtt_state['discovery_sent'] = False
            status = {
                'ts': time.time(),
                'type': 'status',
                'soc_temp': None,
                'tracks': len(tracks),
                'events': len(event_recorders),
            }
            try:
                with open('/sys/class/thermal/thermal_zone0/temp') as _t:
                    status['soc_temp'] = round(int(_t.read().strip()) / 1000.0, 1)
            except Exception:
                pass
            mqtt_publish_status(status, force=True)
            mqtt_publish_recent_events()
            ok = not mqtt_state.get('last_error')
            self.send_response(200 if ok else 500)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            self.wfile.write(json.dumps({
                'ok': ok,
                'last_error': mqtt_state.get('last_error', ''),
                'last_pub': mqtt_state.get('last_pub', 0),
            }).encode())

        elif path == '/api/reboot':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
            threading.Thread(target=lambda: (__import__('time').sleep(0.3), __import__('subprocess').call(['reboot'])), daemon=True).start()

        elif path == '/api/shutdown':
            # Gracefully stop stream_yolo (allows VPSS teardown) then exit.
            # Call this before deploying a new sidecar so VPSS is released cleanly.
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self._cors()
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
            def _do_shutdown():
                import time as _t; import os as _os
                _t.sleep(0.2)
                stop_yolo()
                _t.sleep(2)
                _os.kill(_os.getpid(), 15)
            threading.Thread(target=_do_shutdown, daemon=True).start()

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
    threading.Thread(target=_rss_logger, daemon=True).start()
    threading.Thread(target=_motion_detector, daemon=True).start()
    threading.Thread(target=_storage_cleanup_loop, daemon=True).start()
    threading.Thread(target=_ws_status_loop, daemon=True).start()

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
