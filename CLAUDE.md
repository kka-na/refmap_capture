# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A ROS-free, pure-Python (numpy-only) capture tool for a Livox Mid-360 LiDAR, meant to replace the
original Ubuntu + ROS 2 Humble workflow (`livox_ros_driver2` → `lidar_static_capture` node subscribing
to `/livox/lidar` PointCloud2) when running on macOS, where installing ROS 2 and the Livox driver is
impractical. It talks directly to the lidar over UDP using the Livox SDK2 wire protocol.

- `mac_capture/livox_capture.py` — the entire implementation: command framing/CRC, lidar configuration,
  point-packet parsing, accumulation loop, and PCD writing.
- `for_mac.md` — the original task brief (in Korean) with full protocol reference tables, the
  network/firewall setup procedure, and a troubleshooting guide. Treat it as the primary spec/reference
  doc for this script — read it before making protocol-level changes.
- `captures/` — output of capture runs: `static_scan_<timestamp>/static_scan.pcd` (and, from the
  original ROS workflow, an accompanying `rosbag2/` directory). The Python script only ever writes the
  `.pcd` file, not a rosbag.

## Running

```bash
python3 mac_capture/livox_capture.py --lidar-ip 192.168.1.150 --host-ip 192.168.1.5 \
    --delay 5 --accum 10 --output-dir ~/gtmap_ws/captures
```

Only dependency is `numpy`. Key flags:
- `--delay` — seconds to wait after configuring the lidar before accumulation starts (packets received
  but discarded during this window).
- `--accum` — seconds of points to accumulate into the output PCD.
- `--no-config` — skip sending the host-IP/point-type/work-mode setup commands, for when the lidar is
  already streaming to this host (e.g. re-running after a crash, or when another tool configured it).

There is no test suite, linter, or build step in this repo — it's a single script. Validate changes by
running against the real Mid-360 (IP `192.168.1.150`) or a local UDP simulator that replays the same
command/ACK/point-packet sequence.

### Required host network setup (macOS)

The lidar expects the host at a fixed IP on its subnet, and the host must not be using an overlapping
address range on another interface (e.g. Wi-Fi) or the UDP routing conflicts:

```bash
sudo networksetup -setmanual "<interface>" 192.168.1.5 255.255.255.0
ping 192.168.1.12
```

macOS firewall must allow incoming UDP to the Python process (approve the prompt on first run, or allow
it under System Settings → Network → Firewall). See `for_mac.md`'s troubleshooting section for
ACK-not-arriving / points-not-arriving / packet-loss diagnostics (`lsof -i UDP:56101`, `tcpdump`,
`sysctl kern.ipc.maxsockbuf`).

## Architecture notes

The script implements three layers of the Livox SDK2 protocol directly (no driver, no ROS):

1. **Command channel** (host UDP 56101 → lidar UDP 56100): `pack_cmd()` builds the 24-byte-header +
   payload command frame (CRC16/CCITT-FALSE over the header, CRC32/zlib over the payload). Used once at
   startup via `configure_lidar()` to set the host IP/ports for state, point, and IMU data
   (`KEY_STATE_INFO_HOST_IP`/`KEY_POINT_DATA_HOST_IP`/`KEY_IMU_HOST_IP`), the point data type
   (`KEY_PCL_DATA_TYPE` = cartesian int32 mm), and work mode (`KEY_WORK_MODE` = normal).
   `send_and_wait_ack()` retries with a 0.5s timeout per attempt and validates the ACK's `ret_code`.
2. **Point channel** (lidar → host UDP 56301): `parse_point_packet()` unpacks the 36-byte
   `LivoxLidarEthernetPacket` header then the point array per `_POINT_DTYPES` (data_type 0x01 = int32
   mm, 0x02 = int16 cm). `(0,0,0)` points are filtered out (no-return signal, not a real origin point).
3. **Capture loop** (`main()`): binds the point socket first (4MB `SO_RCVBUF`), optionally configures the
   lidar, then runs a single-threaded poll loop with a monotonic clock: discard until `t0+delay`,
   accumulate raw packet chunks until `t0+delay+accum`, then concatenate and write. This mirrors the
   timeline of the original ROS node exactly (`delay_sec` then `accum_sec`), which downstream consumers
   of the PCD may depend on.

Output format is a binary PCD with fields `x y z intensity` (float32 each), matching what the ROS node
produced — this compatibility is intentional, not incidental, since other tooling in the broader project
consumes PCDs in this exact layout.

If packet loss is a problem (`n_pts` tracked but fewer than the Mid-360's ~200k pts/sec would suggest),
`for_mac.md` suggests splitting the recv loop and parsing into separate threads — this hasn't been done
yet; the current loop does both inline per iteration.
