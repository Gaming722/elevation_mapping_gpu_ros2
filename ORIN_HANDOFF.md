# Jetson AGX Orin deployment handoff

## Repositories and deployment

- Working copy: `<LOCAL_CHECKOUT>`, branch `orin-humble-cu126`.
- Original checkout: `<UPSTREAM_CHECKOUT>`.
- Base commit: upstream `origin/orin_test` at `7ef0411`.
- Verified fork commit: `a4e3ee5` on `Gaming722/elevation_mapping_gpu_ros2:orin-humble-cu126`. It contains the Orin Dockerfile fixes and compatibility documentation.
- SSH alias: `<ORIN_SSH_TARGET>`. The shared machine uses Ubuntu 22.04, ARM64 and L4T R36.5.2. Keep writes inside the dedicated deployment directory and use unique Docker tags. Do not restart or reconfigure existing robot processes as part of a map test.
- Deployment directory: `<ORIN_DEPLOY_DIR>`. Source is under `src/`, and the conversion script is next to it. The successful build log is `rebuild-rosdep.log`.

The Orin image uses ROS Humble, CUDA 12.6 and Python 3.10. The local base image is `humble_cp310:l4t-r36.5.0-pybind11`. The built image is `elevation_mapping:orin-humble`. The derived image `elevation_mapping:orin-humble-cyclone` adds CycloneDDS. To recreate the latter on Orin from this checkout:

```bash
docker build --build-arg ORIN_BASE_IMAGE=elevation_mapping:orin-humble \
  -t elevation_mapping:orin-humble-cyclone - < docker/Dockerfile.orin-cyclone
```

The default image virtual environment points into `/root` and its interpreter is unusable by `ros`. The test runs `/usr/bin/python3` with `/opt/venv/lib/python3.10/site-packages` on `PYTHONPATH`. CuPy 13.6.0 and PyTorch 2.8.0/CUDA 12.6 independently passed GPU arithmetic. The synthetic point cloud and TF launch produced a populated GridMap.

## Real sensor and TF

- The domain 76 navigation stack uses CycloneDDS, `ROS_LOCALHOST_ONLY=0` and a specific network interface in `CYCLONEDDS_URI`. The test reads that setting from the running FAST-LIO process.
- `/livox/lidar` is Livox `CustomMsg`; domain 76 uses frame `lidar_front_168`. The bridge retains the original timestamps, frame, XYZ and reflectivity as intensity. A live test compared five complete converted scans with their originals.
- FAST-LIO publishes `/Odometry` for `camera_init → body`. `odom → camera_init` is identity. `nav_interface_adapter` converts that pose to the chassis frame and publishes `/odom` and `odom → base_link` using the input timestamp. Existing static TF provides `base_link → lidar_front_168`. Reuse it.
- Domain 61 is a different DogLab setup using Fast DDS and localhost discovery. It has used `world → lidar_base` and raw `livox_frame`. Nominal `lidar_base → livox_frame` translation in meters is `[-0.011, -0.02329, 0.04412]` with identity rotation. The static edge passed an isolated timestamp lookup but was never added to the shared DogLab TF graph. Domain 76 does not need that edge.

## Reproduce the 5 Hz test

From the local checkout, with domain 76 FAST-LIO and the Livox stream already running:

```bash
ssh '<ORIN_SSH_TARGET>' \
  'ORIN_DEPLOY_DIR=<ABSOLUTE_DEPLOY_PATH> LIVOX_ROS_SETUP=<ABSOLUTE_LIVOX_SETUP_PATH> bash -s' \
  < tests/orin_domain76_5hz.sh
```

Replace the placeholders with shell-quoted paths as needed. Set `ORIN_TEST_IMAGE` on the remote command if the derived image has a different tag.

The script uses the uploaded bridge, starts a temporary container on the host network and uses separate test topics under `/elevation_orin_test76`. It publishes no robot control commands and starts no navigation or plane segmentation nodes. It waits 0.8 s before querying the exact scan-time TF, then forwards only eligible point clouds at roughly 5 Hz. It uses `map_frame: odom`, `base_frame: base_link`, visibility cleanup enabled, and only `min_filter` and `smooth_filter`. The plugin override and instrumentation live inside the temporary container. The map includes `elevation`, `min_filter` and `smooth`.

The successful run produced 63 GridMap messages, with 48 distinct timestamps on nonempty maps. In the 10 s steady window, output was **5.02 Hz** and distinct map timestamps updated at **3.61 Hz**. Timestamped TF lookup succeeded for **199 of 203** paired scans. The final 8 × 8 m map at 0.04 m resolution held **6,180** finite elevation cells. The first GPU fusion took about six seconds. Map publication and new-scan incorporation are separate measurements.

## Remaining work

- RViz visualization of the real map, sustained full-rate input and 10 Hz output have not been verified. The 5 Hz result used TF-gated, rate-limited input.
- When domain 76 FAST-LIO is off or Livox messages are absent, the test may initialize the map yet produce no output. Check actual messages and matching-time TF, not just topic discovery. The current script can log a missing process environment when FAST-LIO is stopped.
- The bridge sometimes raises an invalid ROS context or external shutdown exception when terminated after testing. This did not affect measured output.
