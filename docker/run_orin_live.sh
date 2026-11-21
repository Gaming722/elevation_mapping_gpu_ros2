#!/usr/bin/env bash
# Run a continuous MID-360 elevation map in the domain 76 navigation graph.
set -e
source /opt/ros/humble/setup.bash
: "${LIVOX_ROS_SETUP:?Set LIVOX_ROS_SETUP to the Livox workspace setup script}"
: "${ORIN_DEPLOY_DIR:?Set ORIN_DEPLOY_DIR to the directory containing the point cloud bridge}"
source "$LIVOX_ROS_SETUP"
cd "$ORIN_DEPLOY_DIR"
export ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -z "${CYCLONEDDS_URI:-}" ]; then
    for pid in $(pgrep -x fastlio_mapping); do
        process_env=$(tr '\000' '\n' < "/proc/$pid/environ" 2>/dev/null) || continue
        if grep -qx 'ROS_DOMAIN_ID=76' <<< "$process_env"; then
            CYCLONEDDS_URI=$(sed -n 's/^CYCLONEDDS_URI=//p' <<< "$process_env")
            break
        fi
    done
    if [ -z "${CYCLONEDDS_URI:-}" ]; then
        echo "Start domain 76 FAST-LIO or set CYCLONEDDS_URI explicitly." >&2
        exit 1
    fi
fi
export CYCLONEDDS_URI
container_name="${ORIN_CONTAINER_NAME:-elevation-orin-live}"
if docker container inspect "$container_name" >/dev/null 2>&1; then
    echo "Container already exists: $container_name" >&2
    exit 1
fi
python3 -B livox_to_pointcloud2.py --output /elevation_orin_test76/livox_points &
bridge=$!
trap 'kill -TERM "$bridge" 2>/dev/null || true; wait "$bridge" || true' EXIT
docker run --name "$container_name" --pull=never --rm -i --runtime=nvidia --network=host \
  -e ROS_DOMAIN_ID -e ROS_LOCALHOST_ONLY -e RMW_IMPLEMENTATION -e CYCLONEDDS_URI \
  --entrypoint /bin/bash "${ORIN_TEST_IMAGE:-elevation_mapping:orin-humble-cyclone}" -s <<'CONTAINER'
set -e
source /opt/ros/humble/setup.bash
source /home/ros/ros_ws/install/setup.bash
export PATH="/usr/bin:$PATH"
export PYTHONPATH="/opt/venv/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
cat >/tmp/domain76.yaml <<'YAML'
/elevation_orin_test76:
  ros__parameters:
    map_frame: odom
    base_frame: base_link
    use_initializer_at_start: false
    enable_visibility_cleanup: true
    service_namespace: /elevation_orin_test76
    subscribers:
      mid360:
        topic_name: /elevation_orin_test76/timed_points
        data_type: pointcloud
    publishers:
      elevation_map:
        layers: [elevation, min_filter, smooth]
        basic_layers: [min_filter]
        fps: 5.0
YAML
sed 's@^/elevation_mapping_node:@/elevation_orin_test76:@' /home/ros/ros_ws/install/share/elevation_mapping_cupy/config/core/core_param.yaml >/tmp/core76.yaml
awk '/^inpainting:|^erosion:/{off=1} off && /enable:/{sub(/True/, "False");off=0} {print}' /home/ros/ros_ws/install/share/elevation_mapping_cupy/config/core/plugin_config.yaml >/tmp/plugins76.yaml
cat >/tmp/instrument76.py <<'PYDIAG'
import elevation_mapping_cupy.elevation_mapping_node as app
class LiveElevationMappingNode(app.ElevationMappingNode):
    def initialize_elevation_mapping(self):
        self.param.plugin_config_file = '/tmp/plugins76.yaml'
        super().initialize_elevation_mapping()

app.ElevationMappingNode = LiveElevationMappingNode
app.main()
PYDIAG
/usr/bin/python3 /tmp/instrument76.py --ros-args -r __node:=elevation_orin_test76 --params-file /tmp/core76.yaml --params-file /tmp/domain76.yaml &
pid=$!
trap 'kill -TERM "$pid" 2>/dev/null || true' EXIT
/usr/bin/python3 -u - <<'PY'
from collections import deque
import math
import time
import rclpy
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

rclpy.init()
node = rclpy.create_node('elevation_domain76_live_gate')
buffer = Buffer()
listener = TransformListener(buffer, node)
pending = deque(maxlen=40)
publisher = node.create_publisher(PointCloud2, '/elevation_orin_test76/timed_points', qos_profile_sensor_data)
latest_map = None

def enqueue(msg):
    pending.append((time.monotonic(), msg))

def receive_map(msg):
    global latest_map
    latest_map = (time.monotonic(), msg)

subscriptions = [
    node.create_subscription(PointCloud2, '/elevation_orin_test76/livox_points', enqueue, qos_profile_sensor_data),
    node.create_subscription(GridMap, '/elevation_orin_test76/elevation_map', receive_map, 10),
]
last_sent = 0.0
last_report = time.monotonic()
sent = 0
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.05)
    now = time.monotonic()
    while pending and now - pending[0][0] >= 0.8:
        _, scan = pending.popleft()
        if now - last_sent < 0.19:
            continue
        try:
            buffer.lookup_transform('odom', scan.header.frame_id, Time.from_msg(scan.header.stamp))
        except TransformException:
            continue
        publisher.publish(scan)
        last_sent = now
        sent += 1
    if now - last_report >= 10:
        if latest_map is None:
            print(f'Waiting for map; forwarded={sent}')
        else:
            received_at, msg = latest_map
            height = msg.data[msg.layers.index('elevation')].data
            finite = sum(math.isfinite(value) for value in height)
            print(f'GridMap frame={msg.header.frame_id} finite={finite} forwarded={sent} age={now-received_at:.2f}s')
        last_report = now
PY
CONTAINER
