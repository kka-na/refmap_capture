#!/usr/bin/env python3
"""ROS-free static-scene capture for a Livox Mid-360 (macOS / Linux / Windows).

Talks to the lidar directly over UDP using the Livox SDK2 protocol, so the
only dependency is numpy.

Timeline (same as the ROS lidar_static_capture node):
  t=0                    lidar configured, packets received but discarded
  t=delay                accumulation starts
  t=delay+accum          accumulated cloud -> PCD (x y z intensity)

Network: give this machine a static IP on the lidar's subnet
(e.g. 192.168.1.5/24) and pass it as --host-ip.
"""
import argparse
import os
import socket
import struct
import sys
import time
import zlib
from datetime import datetime

import numpy as np

# Mid-360 ports (Livox SDK2 sdk_core/comm/define.h)
LIDAR_CMD_PORT = 56100
LIDAR_PUSH_PORT = 56200
LIDAR_POINT_PORT = 56300
LIDAR_IMU_PORT = 56400
HOST_CMD_PORT = 56101
HOST_PUSH_PORT = 56201
HOST_POINT_PORT = 56301
HOST_IMU_PORT = 56401

CMD_WORK_MODE_CONTROL = 0x0100
KEY_PCL_DATA_TYPE = 0x0000
KEY_STATE_INFO_HOST_IP = 0x0005
KEY_POINT_DATA_HOST_IP = 0x0006
KEY_IMU_HOST_IP = 0x0007
KEY_WORK_MODE = 0x001A
WORK_MODE_NORMAL = 0x01

# LivoxLidarEthernetPacket header, 36 bytes before the point data
_PKT_HDR = struct.Struct('<BHHHHBBB12sI8s')
# Point layouts by data_type
_POINT_DTYPES = {
    0x01: (np.dtype([('x', '<i4'), ('y', '<i4'), ('z', '<i4'),
                     ('r', 'u1'), ('tag', 'u1')]), 1e-3),   # mm
    0x02: (np.dtype([('x', '<i2'), ('y', '<i2'), ('z', '<i2'),
                     ('r', 'u1'), ('tag', 'u1')]), 1e-2),   # cm
}


def crc16_ccitt(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pack_cmd(seq, cmd_id, payload):
    """Frame a host->lidar request (SdkPacket in sdk_core/comm/sdk_protocol.h)."""
    length = 24 + len(payload)
    head = struct.pack('<BBHIHBB6s', 0xAA, 0, length, seq, cmd_id, 0, 0, b'\0' * 6)
    crc32 = zlib.crc32(payload) if payload else 0
    return head + struct.pack('<HI', crc16_ccitt(head), crc32) + payload


def kv_payload(items):
    """key_num(u16) + rsvd(u16) + [key(u16) len(u16) value]..."""
    out = struct.pack('<HH', len(items), 0)
    for key, value in items:
        out += struct.pack('<HH', key, len(value)) + value
    return out


def host_ip_value(ip, host_port, lidar_port):
    return socket.inet_aton(ip) + struct.pack('<HH', host_port, lidar_port)


def send_and_wait_ack(sock, lidar_ip, seq, payload, what, retries=5):
    pkt = pack_cmd(seq, CMD_WORK_MODE_CONTROL, payload)
    for _ in range(retries):
        sock.sendto(pkt, (lidar_ip, LIDAR_CMD_PORT))
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            sock.settimeout(max(0.01, deadline - time.monotonic()))
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                break
            if addr[0] != lidar_ip or len(data) < 27 or data[0] != 0xAA:
                continue
            _, _, _, rseq, rcmd, rtype = struct.unpack_from('<BBHIHB', data)
            if rseq != seq or rcmd != CMD_WORK_MODE_CONTROL or rtype != 1:
                continue
            ret_code, error_key = struct.unpack_from('<BH', data, 24)
            if ret_code != 0:
                raise RuntimeError(f'{what}: lidar returned ret_code={ret_code} error_key=0x{error_key:04x}')
            print(f'  {what}: ok')
            return
    raise RuntimeError(f'{what}: no ACK from {lidar_ip}:{LIDAR_CMD_PORT} '
                       '(check cable, --host-ip, and the firewall)')


def configure_lidar(cmd_sock, lidar_ip, host_ip):
    print(f'Configuring lidar {lidar_ip} -> host {host_ip}')
    send_and_wait_ack(cmd_sock, lidar_ip, 1, kv_payload([
        (KEY_STATE_INFO_HOST_IP, host_ip_value(host_ip, HOST_PUSH_PORT, LIDAR_PUSH_PORT)),
        (KEY_POINT_DATA_HOST_IP, host_ip_value(host_ip, HOST_POINT_PORT, LIDAR_POINT_PORT)),
        (KEY_IMU_HOST_IP, host_ip_value(host_ip, HOST_IMU_PORT, LIDAR_IMU_PORT)),
    ]), 'set host ip')
    send_and_wait_ack(cmd_sock, lidar_ip, 2, kv_payload([
        (KEY_PCL_DATA_TYPE, bytes([0x01])),        # cartesian, mm
    ]), 'set point type')
    send_and_wait_ack(cmd_sock, lidar_ip, 3, kv_payload([
        (KEY_WORK_MODE, bytes([WORK_MODE_NORMAL])),
    ]), 'set work mode normal')


def parse_point_packet(data):
    """Return (xyz float32 (N,3) in metres, reflectivity float32 (N,)) or None."""
    if len(data) < _PKT_HDR.size:
        return None
    (_ver, length, _ti, dot_num, _udp_cnt, _frame_cnt, data_type,
     _time_type, _rsvd, _crc32, _ts) = _PKT_HDR.unpack_from(data)
    if data_type not in _POINT_DTYPES:
        return None
    dtype, scale = _POINT_DTYPES[data_type]
    body = data[_PKT_HDR.size:length]
    pts = np.frombuffer(body, dtype=dtype, count=min(dot_num, len(body) // dtype.itemsize))
    xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float32) * scale
    valid = (pts['x'] != 0) | (pts['y'] != 0) | (pts['z'] != 0)   # (0,0,0) = no return
    return xyz[valid], pts['r'][valid].astype(np.float32)


def write_pcd_xyzi(path, points_xyz, intensity):
    points_xyz = np.ascontiguousarray(points_xyz, dtype=np.float32)
    intensity = np.ascontiguousarray(intensity, dtype=np.float32).reshape(-1, 1)
    n = points_xyz.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(np.hstack([points_xyz, intensity]).tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lidar-ip', default='192.168.1.150')
    ap.add_argument('--host-ip', default='192.168.1.5', help="this machine's IP on the lidar subnet")
    ap.add_argument('--delay', type=float, default=5.0, help='seconds to wait before accumulating')
    ap.add_argument('--accum', type=float, default=10.0, help='seconds to accumulate')
    ap.add_argument('--output-dir', default='~/gtmap_ws/captures')
    ap.add_argument('--prefix', default='static_scan')
    ap.add_argument('--no-config', action='store_true',
                    help='do not send config commands (lidar already streams to this host)')
    ap.add_argument('--visualize', action='store_true',
                    help='show a live Open3D window of the cloud as it accumulates (needs open3d; '
                         'closing the window stops the capture early and saves what was collected)')
    args = ap.parse_args()

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = os.path.join(os.path.expanduser(args.output_dir), f'{args.prefix}_{stamp}')
    pcd_path = os.path.join(run_dir, f'{args.prefix}.pcd')

    point_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    point_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    point_sock.bind(('', HOST_POINT_PORT))

    if not args.no_config:
        cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cmd_sock.bind(('', HOST_CMD_PORT))
        configure_lidar(cmd_sock, args.lidar_ip, args.host_ip)
        cmd_sock.close()

    vis = pcd = None
    preview = []        # chunks already folded into the on-screen cloud
    preview_idx = 0      # len(chunks) as of the last preview update
    next_render = 0.0
    view_fitted = False
    if args.visualize:
        import open3d as o3d
        from livox_viewer import intensity_to_rgb
        pcd = o3d.geometry.PointCloud()
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        vis = o3d.visualization.Visualizer()
        vis.create_window('Livox Mid-360 capture', width=1024, height=768)
        vis.add_geometry(pcd)
        vis.add_geometry(axes)
        vis.get_render_option().point_size = 1.5
        vis.get_render_option().background_color = np.array([0.05, 0.05, 0.05])

    print(f'Waiting {args.delay:.1f}s before capture. Keep the scene static from now on.')
    point_sock.settimeout(0.2)
    chunks = []
    n_pkts = n_pts = n_bad = 0
    t0 = time.monotonic()
    accum_start = t0 + args.delay
    accum_end = accum_start + args.accum
    next_status = t0 + 1.0
    got_any = False

    try:
        while True:
            now = time.monotonic()
            if now >= accum_end:
                break
            if vis is not None and not vis.poll_events():
                print('Window closed, saving what was captured so far.')
                break
            if vis is not None and now >= next_render:
                # Throttled: folding in new points and re-rendering every packet would slow the
                # recv loop down enough to drop packets, since parsing runs on the same thread.
                next_render = now + 0.15
                new_chunks = chunks[preview_idx:]
                preview_idx = len(chunks)
                if new_chunks:
                    preview.append((np.concatenate([c[0] for c in new_chunks]),
                                     np.concatenate([c[1] for c in new_chunks])))
                if preview:
                    pcd.points = o3d.utility.Vector3dVector(
                        np.concatenate([p[0] for p in preview]).astype(np.float64))
                    pcd.colors = o3d.utility.Vector3dVector(
                        intensity_to_rgb(np.concatenate([p[1] for p in preview])))
                    vis.update_geometry(pcd)
                    if not view_fitted:
                        vis.reset_view_point(True)
                        view_fitted = True
                vis.update_renderer()
            if now >= next_status:
                next_status += 1.0
                if now < accum_start:
                    print(f'  starting in {accum_start - now:0.1f}s...'
                          + ('' if got_any else '  (no packets yet!)'))
                else:
                    print(f'  recording... {accum_end - now:0.1f}s left ({n_pkts} pkts, {n_pts} pts)')
            try:
                data, addr = point_sock.recvfrom(65535)
            except socket.timeout:
                continue
            if addr[0] != args.lidar_ip:
                continue
            got_any = True
            if time.monotonic() < accum_start:
                continue
            parsed = parse_point_packet(data)
            if parsed is None:
                n_bad += 1
                continue
            chunks.append(parsed)
            n_pkts += 1
            n_pts += parsed[0].shape[0]
    except KeyboardInterrupt:
        print('Interrupted, saving what was captured so far.')
    finally:
        if vis is not None:
            vis.destroy_window()

    if chunks:
        xyz = np.concatenate([c[0] for c in chunks])
        intensity = np.concatenate([c[1] for c in chunks])
    else:
        xyz = np.zeros((0, 3), np.float32)
        intensity = np.zeros((0,), np.float32)

    os.makedirs(run_dir, exist_ok=True)
    write_pcd_xyzi(pcd_path, xyz, intensity)
    print(f'Done. {xyz.shape[0]} points from {n_pkts} packets ({n_bad} dropped).\n  PCD : {pcd_path}')
    if not chunks:
        print('WARNING: no point data received.', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
