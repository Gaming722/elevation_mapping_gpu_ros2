# GridMap web viewer

A ROS 2 subscriber and browser-rendered 3D view. The service listens on IPv4
loopback and serves its own assets. It needs no package downloads, GPU or
browser on the Orin. The client browser needs WebGL 2.

## Offline demo

From the repository root, run `python3 tools/gridmap_web/server.py --demo`
and open <http://127.0.0.1:8765>. The demo shows a moving height surface,
a hole, and the default height scan sampled from the demo GridMap. Drag to orbit, scroll to
zoom, or choose **Top view**.

Use **Lock color scale** to hold the displayed height limits across map
updates and layer switches. Heights outside the range use the nearest
endpoint color. Click again to resume automatic scaling.

## On the Orin

Start or restore the existing map publisher as described in [ORIN_HANDOFF.md](../../ORIN_HANDOFF.md).
The default topic is `/elevation_orin_test76/elevation_map`. Check that the
publisher is producing data. Topic discovery alone is insufficient.

### Use the running map container

Copy the three runtime files (`server.py`, `index.html`, `viewer.js`) to the
Orin, then run the commands below on the Orin. Update the container name
if needed. The live launcher uses host networking, so the container's
loopback listener is reachable through SSH on the Orin host.

```bash
container=elevation-orin-live
docker exec "$container" mktemp -d /tmp/gridmap-web.XXXXXX
```

Set `viewer_path` to the directory printed by the command, then copy and run
the viewer. Run the service in the foreground so Ctrl+C can stop it.

```bash
viewer_path=/tmp/gridmap-web.REPLACE_ME
docker cp tools/gridmap_web/. "$container:$viewer_path/"
docker exec -it "$container" /bin/bash -c '
  source /opt/ros/humble/setup.bash
  source /home/ros/ros_ws/install/setup.bash
  exec /usr/bin/python3 "$@"
' bash "$viewer_path/server.py"
```

You can also run `server.py` directly on the Orin host after sourcing a ROS 2
Humble environment with `rclpy`, `grid_map_msgs` and `tf2_ros`. Match the map
publisher's `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY`
and `CYCLONEDDS_URI`. Set `--topic`, `--layers` or `--base-frame` as
needed. The current live publisher uses `base_link`.

From the **local workstation**:

```bash
ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 <ORIN_SSH_HOST>
```

Open <http://127.0.0.1:8765>. If the local port is occupied, use a different
first port and open that port in the browser.

## Height scan

The viewer samples `elevation` automatically, following
`LY_mjlab`'s M1 actor configuration in `m1_height_scan_env_cfg`.
Its `terrain_scan` inherits the [mjlab 1.6.0 grid](https://github.com/mujocolab/mjlab/blob/v1.6.0/src/mjlab/tasks/velocity/velocity_env_cfg.py#L43-L52).
The footprint spans **1.6 × 1.0 m**, with **0.1 m** spacing and **17 × 11
points**, centered on the trunk. The viewer uses the scan frame's
translation and yaw at the map timestamp and looks up the containing
GridMap cell for each XY position.

Yellow points show absolute terrain height. Red points show misses at
the display reference plane. Missing or out-of-map cells, cells above the
scan frame, and cells more than 5 m below it count as misses. The page
reports the number of valid samples.

Frame metadata includes `scan_shape` as `[ny, nx]` and `height_scan` in
XY order with X varying fastest. Valid entries use
`clip(trunk_z - terrain_z - 0.54, -0.8, 0.8)`. Misses use `0.0` in
`height_scan` and JSON `null` for the absolute Z in `scan`. This is the
deterministic observation from measured cells. Training noise and random
observation delay are excluded.

Optional arguments:

- `--scan-frame trunk` selects the ROS TF frame at the training trunk
  origin, if available. The default is `--base-frame` (`base_link` in the
  Orin launcher). Verify that the frame origin matches the simulated trunk
  before comparing observation values.

- `--scan-layer smooth` samples a different GridMap layer. Changing the
  displayed terrain layer in the browser does not change the scan layer.
- `--preview-scan 15 8 0.10 0.10 0.0 -0.35` changes the footprint to
  1.4 m forward and 0.7 m laterally. The arguments are X count, Y count,
  X spacing, Y spacing, first X, first Y. Use `--scan-offset` to adjust the
  relative-height offset for another policy.
- `--scan-topic /your/height_scan_points` switches to an external
  `sensor_msgs/msg/PointCloud2` with absolute XYZ in meters if available.
  It uses the closest already received scan within `--scan-tolerance`
  seconds of the map stamp (default 0.15). This option requires
  `sensor_msgs_py`.

## Interpreting the metrics

- **Orin receive rate** counts incoming maps over a rolling five-second
  window. **Browser frame rate** counts distinct map frames fetched by
  the browser, independently of the GPU redraw rate.
- **Map age at Orin** compares the map header stamp with the ROS
  subscriber's clock at receipt. Interpret it only when both use the same
  clock. A repeated or old stamp can indicate the publisher is reusing
  an earlier input even when receive rates are healthy.
- **HTTP fetch time** includes request, response and binary body transfer
  through the tunnel. It is a round trip with payload, compared on the
  browser clock. The polling interval is about 65 ms plus the request
  duration. Slow requests can skip maps because the server returns the
  latest frame, without a backlog of old frames.

The service sends one Float32 height layer per update. A 200 × 200 layer
takes about 160 kB per frame before HTTP and SSH overhead, around 0.8 MB/s
at 5 Hz. The browser rotates and renders the mesh locally.
