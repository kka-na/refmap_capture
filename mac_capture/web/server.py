#!/usr/bin/env python3
"""Local web UI for the Livox Mid-360: live view + timed capture + a thumbnail gallery.

Run with: .venv/bin/python3 mac_capture/web/server.py
Then open http://127.0.0.1:8000/ in a browser. Local use only (no auth, binds 127.0.0.1).

A single background thread owns the point UDP socket (same wire protocol as livox_capture.py /
livox_viewer.py) and always keeps a short rolling buffer for the live view. Starting a capture
flips that thread into also accumulating a full-resolution buffer for `delay` + `accum` seconds,
then this module saves a PCD (via livox_capture.write_pcd_xyzi), a meta.json sidecar, and a
thumbnail JPG -- mirroring livox_capture.py's CLI flow, just driven from HTTP requests instead of
argparse.
"""
import json
import os
import queue
import socket
import sys
import threading
import time
from datetime import datetime
from urllib.parse import quote, unquote

import numpy as np
import open3d as o3d
from flask import Flask, Response, abort, jsonify, request, send_from_directory

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MAC_CAPTURE_DIR = os.path.dirname(THIS_DIR)
sys.path.insert(0, MAC_CAPTURE_DIR)

from livox_capture import (  # noqa: E402
    HOST_CMD_PORT, HOST_POINT_PORT, configure_lidar, parse_point_packet, write_pcd_xyzi,
)
from livox_viewer import intensity_to_rgb  # noqa: E402

SETTINGS_PATH = os.path.join(THIS_DIR, 'settings.json')
DEFAULT_SETTINGS = {
    'lidar_ip': '192.168.1.150',
    'host_ip': '192.168.1.5',
    'delay': 5.0,
    'accum': 10.0,
    'output_dir': '/Volumes/Kana_White/Project/refmap_capture/captures',
    'prefix': 'static_scan',
}
LIVE_WINDOW_SEC = 0.3
LIVE_MAX_POINTS = 60000
THUMB_SIZE = (480, 360)


def load_settings():
    if os.path.isfile(SETTINGS_PATH):
        try:
            with open(SETTINGS_PATH) as f:
                saved = json.load(f)
            return {**DEFAULT_SETTINGS, **saved}
        except Exception:
            pass
    return dict(DEFAULT_SETTINGS)


def save_settings(params):
    settings = {k: params[k] for k in DEFAULT_SETTINGS if k in params}
    with open(SETTINGS_PATH, 'w') as f:
        json.dump(settings, f, indent=2)


class LidarStream:
    """Owns the point-data UDP socket. Always feeds a short rolling buffer for the live view;
    while `capturing` also accumulates a second, unbounded buffer for the active capture."""

    def __init__(self, lidar_ip):
        self.lidar_ip = lidar_ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind(('', HOST_POINT_PORT))
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self.live_chunks = []       # [(monotonic_ts, xyz, intensity)], rolling window
        self.capture_chunks = None  # None when idle, else a growing list of (xyz, intensity)
        self.last_packet_time = 0.0
        self._stop = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self._stop:
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                data = None
            except OSError:
                break

            parsed = None
            if data is not None and addr[0] == self.lidar_ip:
                parsed = parse_point_packet(data)
                if parsed is not None and parsed[0].shape[0] == 0:
                    parsed = None

            # Always age out the rolling buffer, even on a timeout with no packet -- otherwise
            # a dropped lidar connection leaves the last snapshot frozen and /api/live looks
            # "live" forever instead of correctly going empty.
            now = time.monotonic()
            with self.lock:
                if parsed is not None:
                    self.live_chunks.append((now, parsed[0], parsed[1]))
                    self.last_packet_time = now
                    if self.capture_chunks is not None:
                        self.capture_chunks.append(parsed)
                cutoff = now - LIVE_WINDOW_SEC
                self.live_chunks = [c for c in self.live_chunks if c[0] >= cutoff]

    def live_snapshot(self):
        with self.lock:
            chunks = list(self.live_chunks)
        if not chunks:
            return np.zeros((0, 3), np.float32), np.zeros((0,), np.float32)
        xyz = np.concatenate([c[1] for c in chunks])
        intensity = np.concatenate([c[2] for c in chunks])
        if xyz.shape[0] > LIVE_MAX_POINTS:
            idx = np.random.choice(xyz.shape[0], LIVE_MAX_POINTS, replace=False)
            xyz, intensity = xyz[idx], intensity[idx]
        return xyz, intensity

    def is_connected(self):
        with self.lock:
            last = self.last_packet_time
        return (time.monotonic() - last) < 1.0

    def begin_capture(self):
        with self.lock:
            self.capture_chunks = []

    def capture_progress(self):
        with self.lock:
            chunks = self.capture_chunks or []
            return len(chunks), sum(c[0].shape[0] for c in chunks)

    def end_capture(self):
        with self.lock:
            chunks = self.capture_chunks
            self.capture_chunks = None
        if not chunks:
            return np.zeros((0, 3), np.float32), np.zeros((0,), np.float32), 0
        xyz = np.concatenate([c[0] for c in chunks])
        intensity = np.concatenate([c[1] for c in chunks])
        return xyz, intensity, len(chunks)

    def stop(self):
        self._stop = True
        self.sock.close()


def _render_thumbnail_impl(xyz, intensity, run_dir):
    """Renders a fixed isometric snapshot of the cloud. Returns the thumbnail filename, or None.

    Must only ever run on the process's real main thread: Open3D's OffscreenRenderer initializes a
    Cocoa NSApplication under the hood on macOS, and calling that off the main thread is an
    instant EXC_BREAKPOINT crash (confirmed via a crash report during testing). See
    `render_thumbnail()` below, which is the thread-safe entry point every caller should use.
    """
    if xyz.shape[0] == 0:
        return None
    if xyz.shape[0] > 200_000:
        idx = np.random.choice(xyz.shape[0], 200_000, replace=False)
        xyz, intensity = xyz[idx], intensity[idx]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(intensity_to_rgb(intensity))

    renderer = o3d.visualization.rendering.OffscreenRenderer(*THUMB_SIZE)
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = 'defaultUnlit'
    mat.point_size = 2.0
    renderer.scene.add_geometry('pcd', pcd, mat)
    renderer.scene.set_background([0.05, 0.05, 0.05, 1.0])
    bbox = pcd.get_axis_aligned_bounding_box()
    center = bbox.get_center()
    extent = max(float(np.linalg.norm(bbox.get_extent())), 1.0)
    eye = center + np.array([1.0, -1.0, 0.8]) * extent
    renderer.setup_camera(60.0, center, eye, [0, 0, 1])
    img = renderer.render_to_image()

    out_path = os.path.join(run_dir, 'thumb.jpg')
    if not o3d.io.write_image(out_path, img):
        out_path = os.path.join(run_dir, 'thumb.png')
        o3d.io.write_image(out_path, img)
    return os.path.basename(out_path)


_render_queue = queue.Queue()


def render_thumbnail(xyz, intensity, run_dir):
    """Thread-safe entry point: queues the render job and blocks until the main-thread worker
    (started in __main__ below) has handled it. Call this from any thread; never call
    `_render_thumbnail_impl` directly."""
    done = threading.Event()
    result = {}
    _render_queue.put((xyz, intensity, run_dir, result, done))
    if not done.wait(timeout=30):
        raise RuntimeError('thumbnail render timed out')
    if 'error' in result:
        raise RuntimeError(result['error'])
    return result.get('thumb')


def _render_worker():
    """Drains the thumbnail render queue. Must run on the process main thread (see
    `_render_thumbnail_impl`'s docstring)."""
    while True:
        xyz, intensity, run_dir, result, done = _render_queue.get()
        try:
            result['thumb'] = _render_thumbnail_impl(xyz, intensity, run_dir)
        except Exception as e:
            result['error'] = str(e)
        finally:
            done.set()


def read_pcd_point_count(path):
    try:
        with open(path, 'rb') as f:
            for _ in range(15):
                line = f.readline()
                if line.startswith(b'POINTS'):
                    return int(line.split()[1])
    except OSError:
        pass
    return 0


def ensure_thumbnail(run_dir, pcd_path):
    for name in ('thumb.jpg', 'thumb.png'):
        if os.path.isfile(os.path.join(run_dir, name)):
            return name
    try:
        pcd = o3d.io.read_point_cloud(pcd_path)
        xyz = np.asarray(pcd.points, dtype=np.float32)
        # Our PCD writer doesn't store a color field Open3D recognizes, so for pre-existing
        # captures that predate meta.json/thumbnails we just render them uncolored (mid-gray).
        intensity = np.full((xyz.shape[0],), 40.0, dtype=np.float32)
        return render_thumbnail(xyz, intensity, run_dir)
    except Exception as e:
        print(f'thumbnail generation failed for {pcd_path}: {e}', file=sys.stderr)
        return None


def list_captures(output_dir):
    output_dir = os.path.expanduser(output_dir)
    if not os.path.isdir(output_dir):
        return []
    entries = []
    for name in sorted(os.listdir(output_dir)):
        run_dir = os.path.join(output_dir, name)
        if not os.path.isdir(run_dir):
            continue
        pcds = [f for f in os.listdir(run_dir) if f.endswith('.pcd')]
        if not pcds:
            continue
        pcd_path = os.path.join(run_dir, pcds[0])
        meta = {}
        meta_path = os.path.join(run_dir, 'meta.json')
        if os.path.isfile(meta_path):
            try:
                with open(meta_path) as f:
                    meta = json.load(f)
            except Exception:
                meta = {}
        timestamp = meta.get('start_time') or datetime.fromtimestamp(
            os.path.getmtime(pcd_path)).isoformat()
        point_count = meta.get('point_count')
        if point_count is None:
            point_count = read_pcd_point_count(pcd_path)
        thumb_name = ensure_thumbnail(run_dir, pcd_path)
        entries.append({
            'name': name,
            'timestamp': timestamp,
            'point_count': point_count,
            'delay': meta.get('delay'),
            'accum': meta.get('accum'),
            'thumb_url': f'/api/thumbnail?path={quote(run_dir)}' if thumb_name else None,
        })
    entries.sort(key=lambda e: e['timestamp'], reverse=True)
    return entries


app = Flask(__name__, static_folder=os.path.join(THIS_DIR, 'static'), static_url_path='')

settings = load_settings()
stream = LidarStream(settings['lidar_ip'])
try:
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.bind(('', HOST_CMD_PORT))
    configure_lidar(cmd_sock, settings['lidar_ip'], settings['host_ip'])
    cmd_sock.close()
except Exception as e:
    print(f'WARNING: could not configure lidar at startup: {e}', file=sys.stderr)

capture_lock = threading.Lock()
capture_status = {'state': 'idle', 'elapsed': 0.0, 'remaining': 0.0, 'pkts': 0, 'pts': 0,
                   'error': None, 'last_dir': None}


def run_capture(params):
    delay = float(params['delay'])
    accum = float(params['accum'])
    output_dir = os.path.expanduser(params['output_dir'])
    prefix = params.get('prefix') or 'static_scan'
    start_time = datetime.now()
    run_dir = os.path.join(output_dir, f"{prefix}_{start_time.strftime('%Y%m%d_%H%M%S')}")

    with capture_lock:
        capture_status.update(state='waiting', elapsed=0.0, remaining=delay + accum,
                               pkts=0, pts=0, error=None)
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < delay:
            elapsed = time.monotonic() - t0
            with capture_lock:
                capture_status.update(elapsed=elapsed, remaining=delay + accum - elapsed)
            time.sleep(0.1)

        stream.begin_capture()
        with capture_lock:
            capture_status.update(state='recording')
        accum_start = time.monotonic()
        while time.monotonic() - accum_start < accum:
            pkts, pts = stream.capture_progress()
            elapsed = time.monotonic() - t0
            with capture_lock:
                capture_status.update(elapsed=elapsed, remaining=delay + accum - elapsed,
                                       pkts=pkts, pts=pts)
            time.sleep(0.15)
        xyz, intensity, n_pkts = stream.end_capture()

        os.makedirs(run_dir, exist_ok=True)
        pcd_path = os.path.join(run_dir, f'{prefix}.pcd')
        write_pcd_xyzi(pcd_path, xyz, intensity)

        meta = {
            'start_time': start_time.isoformat(),
            'lidar_ip': params['lidar_ip'], 'host_ip': params['host_ip'],
            'delay': delay, 'accum': accum, 'prefix': prefix,
            'point_count': int(xyz.shape[0]), 'packet_count': int(n_pkts),
        }
        with open(os.path.join(run_dir, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=2)

        try:
            render_thumbnail(xyz, intensity, run_dir)
        except Exception as e:
            print(f'thumbnail generation failed: {e}', file=sys.stderr)

        save_settings(params)
        with capture_lock:
            capture_status.update(state='done', elapsed=delay + accum, remaining=0.0,
                                   pkts=n_pkts, pts=int(xyz.shape[0]), last_dir=run_dir)
    except Exception as e:
        stream.end_capture()  # make sure we don't leave the stream stuck in capturing mode
        with capture_lock:
            capture_status.update(state='error', error=str(e))


@app.route('/')
def index():
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/api/live')
def api_live():
    xyz, intensity = stream.live_snapshot()
    n = xyz.shape[0]
    rgb = (np.clip(intensity_to_rgb(intensity), 0, 1) * 255).astype(np.uint8) if n else \
        np.zeros((0, 3), np.uint8)
    # 8-byte header (point count, connected flag) keeps the position array's byte offset a
    # multiple of 4, which Float32Array on the JS side requires.
    header = np.array([n, 1 if stream.is_connected() else 0], dtype='<u4').tobytes()
    body = xyz.astype('<f4').tobytes() + rgb.tobytes()
    return Response(header + body, mimetype='application/octet-stream')


@app.route('/api/settings', methods=['GET', 'POST'])
def api_settings():
    global settings
    if request.method == 'POST':
        body = request.get_json(force=True)
        settings = {**settings, **{k: body[k] for k in DEFAULT_SETTINGS if k in body}}
        save_settings(settings)
    return jsonify(settings)


@app.route('/api/capture/start', methods=['POST'])
def api_capture_start():
    with capture_lock:
        if capture_status['state'] in ('waiting', 'recording'):
            return jsonify({'error': 'a capture is already running'}), 409
    body = request.get_json(force=True)
    params = {**settings, **{k: body[k] for k in DEFAULT_SETTINGS if k in body}}
    threading.Thread(target=run_capture, args=(params,), daemon=True).start()
    return jsonify({'started': True})


@app.route('/api/capture/status')
def api_capture_status():
    with capture_lock:
        return jsonify(dict(capture_status))


@app.route('/api/captures')
def api_captures():
    output_dir = request.args.get('dir', settings['output_dir'])
    return jsonify(list_captures(output_dir))


@app.route('/api/thumbnail')
def api_thumbnail():
    run_dir = unquote(request.args.get('path', ''))
    if not run_dir or not os.path.isdir(run_dir):
        abort(404)
    for name in ('thumb.jpg', 'thumb.png'):
        if os.path.isfile(os.path.join(run_dir, name)):
            return send_from_directory(run_dir, name)
    abort(404)


@app.route('/api/browse')
def api_browse():
    """Lists subdirectories of a path, for the output-folder picker modal."""
    path = os.path.expanduser(request.args.get('path') or DEFAULT_SETTINGS['output_dir'])
    if not os.path.isdir(path):
        path = os.path.expanduser('~')
    path = os.path.abspath(path)
    try:
        dirs = sorted(
            name for name in os.listdir(path)
            if not name.startswith('.') and os.path.isdir(os.path.join(path, name))
        )
    except OSError:
        dirs = []
    parent = os.path.dirname(path)
    return jsonify({'path': path, 'parent': parent if parent != path else None, 'dirs': dirs})


if __name__ == '__main__':
    # Flask runs on a background thread so the real process main thread is free to run the
    # thumbnail render worker -- required on macOS, see _render_thumbnail_impl's docstring.
    flask_thread = threading.Thread(
        target=lambda: app.run(host='127.0.0.1', port=8000, threaded=True, use_reloader=False),
        daemon=True)
    flask_thread.start()
    print('Serving on http://127.0.0.1:8000/')
    try:
        _render_worker()
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop()
