#!/usr/bin/env python3
"""Serve the latest ROS GridMap over loopback HTTP."""
import argparse
from array import array
from collections import deque
from copy import copy
import json
import math
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


def decode_layer(layer, nx, ny, outer, inner):
    """Return little-endian floats with X as the fastest varying index."""
    dims = [(d.label, d.size) for d in layer.layout.dim]
    offset = layer.layout.data_offset
    if nx < 1 or ny < 1 or len(layer.data) != offset + nx * ny:
        raise ValueError("Incomplete GridMap layer")
    if dims == [("column_index", ny), ("row_index", nx)] and not (offset or outer or inner):
        values = array("f", layer.data)
        if sys.byteorder != "little":
            values.byteswap()
        return values.tobytes()
    if dims == [("column_index", ny), ("row_index", nx)]:
        values = (layer.data[offset + ((y + inner) % ny) * nx + (x + outer) % nx]
                  for y in range(ny) for x in range(nx))
    elif dims == [("row_index", nx), ("column_index", ny)]:
        values = (layer.data[offset + ((x + outer) % nx) * ny + (y + inner) % ny]
                  for y in range(ny) for x in range(nx))
    else:
        raise ValueError(f"Unsupported GridMap layout {dims}")
    return struct.pack(f"<{nx * ny}f", *values)


def transform_point(transform, point):
    q, t = transform.rotation, transform.translation
    x, y, z = point
    a = 2 * (q.y * z - q.z * y)
    b = 2 * (q.z * x - q.x * z)
    c = 2 * (q.x * y - q.y * x)
    return (x + q.w * a + q.y * c - q.z * b + t.x,
            y + q.w * b + q.z * a - q.x * c + t.y,
            z + q.w * c + q.x * b - q.y * a + t.z)


# LY_mjlab M1 actor's terrain_scan inherits the mjlab 1.6.0 grid and ray range.
M1_SCAN_GRID = (17, 11, 0.1, 0.1, -0.8, -0.5)
M1_SCAN_MAX_DISTANCE = 5.0
M1_SCAN_CLIP = 0.8


def sample_height_scan(spec, transform, meta, data, offset):
    """Sample containing cells in mjlab XY order with a yaw-aligned footprint."""
    count_x, count_y, dx, dy, first_x, first_y = spec
    q, t = transform.rotation, transform.translation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    nx, ny, resolution = meta["nx"], meta["ny"], meta["resolution"]
    cx, cy = meta["center"]
    points, values = [], []
    for j in range(count_y):
        local_y = first_y + j * dy
        for i in range(count_x):
            local_x = first_x + i * dx
            x = t.x + cos_yaw * local_x - sin_yaw * local_y
            y = t.y + sin_yaw * local_x + cos_yaw * local_y
            ix = math.floor(nx / 2 + (cx - x) / resolution)
            iy = math.floor(ny / 2 + (cy - y) / resolution)
            z = None
            if 0 <= ix < nx and 0 <= iy < ny:
                value = struct.unpack_from("<f", data, 4 * (iy * nx + ix))[0]
                if math.isfinite(value) and 0 <= t.z - value <= M1_SCAN_MAX_DISTANCE:
                    z = value
            points.append([x, y, z])
            observation = 0.0 if z is None else t.z - z - offset
            values.append(max(-M1_SCAN_CLIP, min(M1_SCAN_CLIP, observation)))
    return points, values


def ros_source(args, state, stop):
    import rclpy
    from grid_map_msgs.msg import GridMap
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    def stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def on_scan(msg):
        try:
            rows = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
            scans.append((msg.header, [(float(x), float(y), float(z)) for x, y, z in rows]))
        except (AssertionError, ValueError, KeyError) as exc:
            node.get_logger().error(f"Invalid height scan: {exc}")

    def lookup(frame, source, stamp):
        return tf.lookup_transform(frame, source, Time.from_msg(stamp)).transform

    def on_map(msg):
        now = time.monotonic()
        rates.append(now)
        while rates and now - rates[0] > 5:
            rates.popleft()
        stamp = msg.header.stamp
        ns = stamp_ns(stamp)
        frame = msg.header.frame_id
        info = msg.info
        if not math.isfinite(info.resolution) or info.resolution <= 0:
            raise ValueError("GridMap resolution must be positive")
        if not all(map(math.isfinite, (info.pose.position.x, info.pose.position.y))):
            raise ValueError("GridMap center must be finite")
        nx, ny = round(info.length_x / info.resolution), round(info.length_y / info.resolution)
        layers = {name: decode_layer(layer, nx, ny, msg.outer_start_index, msg.inner_start_index)
                  for name, layer in zip(msg.layers, msg.data)
                  if name in args.layers or (not args.scan_topic and name == args.scan_layer)}
        display_layers = {name: layers[name] for name in args.layers if name in layers}
        if not display_layers:
            raise ValueError("No requested display layers in GridMap")
        meta = dict(nx=nx, ny=ny, resolution=info.resolution, frame=frame,
                    center=[info.pose.position.x, info.pose.position.y],
                    stamp=f"{stamp.sec}.{stamp.nanosec:09d}",
                    age_ms=(node.get_clock().now().nanoseconds - ns) / 1e6 if ns else None,
                    hz=(len(rates) - 1) / (now - rates[0]) if len(rates) > 1 else None,
                    scan_status="", scan=[], robot=None)
        robot = None
        try:
            robot = lookup(frame, args.base_frame, stamp)
            start = transform_point(robot, (0, 0, 0))
            end = transform_point(robot, (0.35, 0, 0))
            meta["robot"] = [start, end]
        except TransformException:
            pass
        if args.scan_topic:
            meta["scan_status"] = "Waiting for a time-aligned scan"
            if scans:
                header, points = min(scans, key=lambda entry: abs(stamp_ns(entry[0].stamp) - ns))
                if abs(stamp_ns(header.stamp) - ns) <= args.scan_tolerance * 1e9:
                    try:
                        tf_scan = lookup(frame, header.frame_id, header.stamp) if header.frame_id != frame else None
                        for x, y, z in points:
                            if not (math.isfinite(x) and math.isfinite(y)):
                                continue
                            valid = math.isfinite(z)
                            point = transform_point(tf_scan, (x, y, z if valid else 0)) if tf_scan else (x, y, z)
                            meta["scan"].append([point[0], point[1], point[2] if valid else None])
                        meta["scan_status"] = "Policy scan · PointCloud2"
                    except TransformException:
                        meta["scan_status"] = "Scan transform unavailable"
        else:
            if args.scan_layer in layers:
                scan_tf = robot
                if args.scan_frame != args.base_frame:
                    try:
                        scan_tf = lookup(frame, args.scan_frame, stamp)
                    except TransformException:
                        scan_tf = None
                if scan_tf is None:
                    meta["scan_status"] = f"GridMap scan · waiting for map-time TF to {args.scan_frame}"
                else:
                    set_gridmap_scan(meta, args, scan_tf, layers[args.scan_layer])
            else:
                meta["scan_status"] = f"GridMap scan · layer {args.scan_layer} unavailable"
        state["frame"] = (state["seq"] + 1, now, meta, display_layers)
        state["seq"] += 1
        state["error"] = None

    def receive(msg):
        try:
            on_map(msg)
        except (ValueError, ZeroDivisionError, OverflowError) as exc:
            state["error"] = str(exc)

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("gridmap_web")
    try:
        tf = Buffer()
        TransformListener(tf, node)
        rates, scans = deque(), deque(maxlen=20)
        grid_qos = copy(qos_profile_sensor_data)
        grid_qos.depth = 1
        node.create_subscription(GridMap, args.topic, receive, grid_qos)
        if args.scan_topic:
            from sensor_msgs.msg import PointCloud2
            from sensor_msgs_py import point_cloud2
            node.create_subscription(PointCloud2, args.scan_topic, on_scan, qos_profile_sensor_data)
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def set_gridmap_scan(meta, args, robot, layer):
    grid = args.preview_scan or M1_SCAN_GRID
    meta["scan"], meta["height_scan"] = sample_height_scan(grid, robot, meta, layer, args.scan_offset)
    meta["scan_shape"] = [grid[1], grid[0]]
    meta["scan_status"] = f"GridMap scan · {args.scan_layer} · {grid[0]} × {grid[1]} · {args.scan_frame}"


def demo_source(args, state, stop):
    from types import SimpleNamespace

    robot = SimpleNamespace(rotation=SimpleNamespace(w=1, x=0, y=0, z=0),
                            translation=SimpleNamespace(x=0, y=0, z=0.5))
    nx, ny, res = 80, 64, 0.055
    for seq in range(1, 2**53):
        phase = seq * 0.12
        values = [math.sin(x / 11 + phase) * 0.13 + math.cos(y / 10) * 0.12
                  + (0.22 if x > 57 else 0)
                  if (x - 49)**2 + (y - 39)**2 > 30 and not (x < 5 and y > 32) else math.nan
                  for y in range(ny) for x in range(nx)]
        meta = dict(nx=nx, ny=ny, resolution=res, center=[0, 0], frame="demo",
                    stamp="—", age_ms=None, hz=5, scan=[], scan_status="Simulated map",
                    robot=[[0, 0, 0.5], [0.35, 0, 0.5]])
        fmt = f"<{len(values)}f"
        layers = {"elevation": struct.pack(fmt, *values),
                  "smooth": struct.pack(fmt, *(z * 0.85 for z in values))}
        if not args.scan_topic and args.scan_layer in layers:
            set_gridmap_scan(meta, args, robot, layers[args.scan_layer])
            meta["scan_status"] = "Demo · " + meta["scan_status"]
        state["frame"] = (seq, time.monotonic(), meta, layers)
        if stop.wait(0.2):
            return


def main():
    from urllib.parse import parse_qs, urlsplit

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--topic", default="/elevation_orin_test76/elevation_map")
    parser.add_argument("--layers", nargs="+", default=["elevation", "min_filter", "smooth"])
    parser.add_argument("--base-frame", default="base_link")
    scan = parser.add_mutually_exclusive_group()
    scan.add_argument("--scan-topic", help="PointCloud2 containing actual policy samples in meters")
    scan.add_argument("--preview-scan", nargs=6, type=float, metavar=("NX", "NY", "DX", "DY", "X0", "Y0"))
    parser.add_argument("--scan-layer", default="elevation", help="GridMap layer sampled by the viewer")
    parser.add_argument("--scan-frame", help="TF frame at the training trunk origin (defaults to --base-frame)")
    parser.add_argument("--scan-offset", type=float, default=0.54, help="Height observation offset in meters")
    parser.add_argument("--scan-tolerance", type=float, default=0.15)
    parser.add_argument("--demo", action="store_true")
    args = parser.parse_args()
    args.scan_frame = args.scan_frame or args.base_frame
    if not math.isfinite(args.scan_offset):
        parser.error("Scan offset must be finite")
    if not math.isfinite(args.scan_tolerance) or args.scan_tolerance < 0:
        parser.error("Scan tolerance must be finite and nonnegative")
    if args.preview_scan:
        nx, ny, dx, dy, x0, y0 = args.preview_scan
        if not all(map(math.isfinite, args.preview_scan)) or not (
            nx.is_integer() and ny.is_integer() and nx >= 1 and ny >= 1 and nx * ny <= 10000 and dx > 0 and dy > 0
        ):
            parser.error("Invalid preview scan counts or spacing")
        args.preview_scan = (int(nx), int(ny), dx, dy, x0, y0)

    state = {"frame": None, "seq": 0, "error": None}
    stop = threading.Event()
    assets = Path(__file__).resolve().parent

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def send(self, status, content_type, body=b""):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            url = urlsplit(self.path)
            files = {"/": ("index.html", "text/html"), "/viewer.js": ("viewer.js", "text/javascript")}
            if url.path in files:
                filename, mime = files[url.path]
                self.send(200, mime, (assets / filename).read_bytes())
                return
            if url.path != "/frame":
                self.send_error(404)
                return
            frame = state["frame"]
            if frame is None or state["error"]:
                self.send(503, "text/plain", (state["error"] or "Waiting for GridMap").encode())
                return
            seq, received, meta, layers = frame
            query = parse_qs(url.query)
            layer = query.get("layer", [""])[0]
            if layer not in layers:
                layer = next(iter(layers))
            if query.get("since") == [str(seq)] and query.get("layer") == [layer]:
                self.send(204, "text/plain")
                return
            metadata = json.dumps(dict(meta, seq=seq, layer=layer, layers=list(layers),
                                       held_ms=1000 * (time.monotonic() - received)),
                                  allow_nan=False).encode()
            self.send(200, "application/octet-stream",
                      struct.pack("<I", len(metadata)) + metadata + layers[layer])

    def source():
        try:
            if args.demo:
                demo_source(args, state, stop)
            else:
                ros_source(args, state, stop)
        except Exception as exc:
            state["error"] = str(exc)
            raise

    with ThreadingHTTPServer(("127.0.0.1", args.port), Handler) as server:
        worker = threading.Thread(target=source, daemon=True)
        worker.start()
        print(f"http://127.0.0.1:{server.server_port}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            worker.join(timeout=2)


if __name__ == "__main__":
    main()
