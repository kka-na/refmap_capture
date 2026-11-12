#!/usr/bin/env python3
"""Real-time point-cloud viewer for a Livox Mid-360 (ROS-free, same UDP path as livox_capture.py).

Opens an Open3D window and continuously refreshes it with the latest sliding
window of points received from the lidar. Not a capture tool: nothing is
saved to disk. Use livox_capture.py for that.
"""
import argparse
import time

import numpy as np
import open3d as o3d

from livox_capture import (
    HOST_CMD_PORT, HOST_POINT_PORT, configure_lidar, parse_point_packet,
)
import socket

# Hand-rolled turbo-ish colormap (blue -> cyan -> green -> yellow -> red) so low-reflectivity
# indoor points still stand out against the dark background instead of fading to near-black gray.
_CMAP_STOPS = np.array([
    [0.19, 0.07, 0.23],
    [0.27, 0.41, 0.88],
    [0.16, 0.76, 0.76],
    [0.48, 0.90, 0.29],
    [0.96, 0.84, 0.17],
    [0.90, 0.19, 0.08],
])


def intensity_to_rgb(intensity, lo=2.0, hi=80.0):
    """Percentile-ish fixed stretch: Mid-360 indoor reflectivity mostly sits well under 255."""
    t = np.clip((intensity - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    n = _CMAP_STOPS.shape[0] - 1
    pos = t * n
    idx = np.clip(pos.astype(np.int32), 0, n - 1)
    frac = (pos - idx)[:, None]
    return _CMAP_STOPS[idx] * (1 - frac) + _CMAP_STOPS[idx + 1] * frac


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--lidar-ip', default='192.168.1.150')
    ap.add_argument('--host-ip', default='192.168.1.5', help="this machine's IP on the lidar subnet")
    ap.add_argument('--window', type=float, default=0.2, help='seconds of points kept on screen (rolling)')
    ap.add_argument('--point-size', type=float, default=1.5)
    ap.add_argument('--no-config', action='store_true',
                     help='do not send config commands (lidar already streams to this host)')
    args = ap.parse_args()

    point_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    point_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    point_sock.bind(('', HOST_POINT_PORT))
    point_sock.settimeout(0.05)

    if not args.no_config:
        cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cmd_sock.bind(('', HOST_CMD_PORT))
        configure_lidar(cmd_sock, args.lidar_ip, args.host_ip)
        cmd_sock.close()

    pcd = o3d.geometry.PointCloud()
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    vis = o3d.visualization.Visualizer()
    vis.create_window('Livox Mid-360 live', width=1024, height=768)
    vis.add_geometry(pcd)
    vis.add_geometry(axes)
    vis.get_render_option().point_size = args.point_size
    vis.get_render_option().background_color = np.array([0.05, 0.05, 0.05])

    print('Streaming... close the window to stop.')
    chunks = []  # list of (timestamp, xyz, intensity)
    total_pkts = total_pts = other_src_pkts = 0
    settled = False  # keep auto-fitting the camera until we've seen a real frame
    t_start = time.monotonic()
    next_status = t_start + 1.0
    try:
        while True:
            try:
                data, addr = point_sock.recvfrom(65535)
            except socket.timeout:
                data = None
            if data is not None:
                if addr[0] != args.lidar_ip:
                    other_src_pkts += 1
                else:
                    parsed = parse_point_packet(data)
                    if parsed is not None and parsed[0].shape[0] > 0:
                        chunks.append((time.monotonic(), parsed[0], parsed[1]))
                        total_pkts += 1
                        total_pts += parsed[0].shape[0]

            now = time.monotonic()
            cutoff = now - args.window
            chunks = [c for c in chunks if c[0] >= cutoff]

            if now >= next_status:
                next_status += 1.0
                print(f'  total: {total_pkts} pkts / {total_pts} pts from {args.lidar_ip}'
                      + (f', {other_src_pkts} pkts from other sources (ignored)' if other_src_pkts else '')
                      + f' | in view now: {sum(c[1].shape[0] for c in chunks)} pts')

            if chunks:
                xyz = np.concatenate([c[1] for c in chunks])
                intensity = np.concatenate([c[2] for c in chunks])
                pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
                pcd.colors = o3d.utility.Vector3dVector(intensity_to_rgb(intensity))
                vis.update_geometry(pcd)
                if not settled:
                    vis.reset_view_point(True)
                    if now - t_start > 1.0:   # a second of real frames in view -> good fit, stop re-fitting
                        settled = True

            if not vis.poll_events():
                break
            vis.update_renderer()
    except KeyboardInterrupt:
        pass
    finally:
        vis.destroy_window()
        point_sock.close()
        if total_pkts == 0:
            print('WARNING: no point packets ever received. Check lidar-ip/host-ip, cabling, '
                  'and the macOS firewall prompt for this python binary.', file=__import__('sys').stderr)


if __name__ == '__main__':
    main()
