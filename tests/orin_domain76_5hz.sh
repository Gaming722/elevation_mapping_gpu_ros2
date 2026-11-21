set -e
source /opt/ros/humble/setup.bash
: "${LIVOX_ROS_SETUP:?Set LIVOX_ROS_SETUP to the Livox workspace setup script}"
: "${ORIN_DEPLOY_DIR:?Set ORIN_DEPLOY_DIR to the directory containing the point cloud bridge}"
source "$LIVOX_ROS_SETUP"
cd "$ORIN_DEPLOY_DIR"
export ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
p=$(pgrep -x fastlio_mapping | head -1)
v=$(tr '\000' '\n' < "/proc/$p/environ" | sed -n 's/^CYCLONEDDS_URI=//p')
if [ -n "$v" ]; then export CYCLONEDDS_URI="$v"; fi
python3 -B livox_to_pointcloud2.py --output /elevation_orin_test76/livox_points &
bridge=$!
trap 'kill -TERM "$bridge" 2>/dev/null || true; wait "$bridge" || true' EXIT
docker run --pull=never --rm -i --runtime=nvidia --network=host \
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
class Probe(app.ElevationMappingNode):
    def __init__(self):
        self.inputs=0
        super().__init__()
        self.create_timer(2.0,self.report)
    def initialize_elevation_mapping(self):
        self.param.plugin_config_file='/tmp/plugins76.yaml'
        super().initialize_elevation_mapping()
    def pointcloud_callback(self,msg,sub_key):
        self.inputs+=1
        if self.inputs<=2:
            self.get_logger().info(f"ENTER cloud {msg.width} {msg.header.frame_id}")
        super().pointcloud_callback(msg,sub_key)
        if self.inputs<=2:
            self.get_logger().info(f"EXIT cloud processed={self._pointcloud_process_counter}")
    def report(self):
        self.get_logger().info(f"probe inputs={self.inputs}, integrated={self._pointcloud_process_counter}, pose={self._map_q is not None}")
app.ElevationMappingNode=Probe
app.main()
PYDIAG
/usr/bin/python3 /tmp/instrument76.py --ros-args -r __node:=elevation_orin_test76 --params-file /tmp/core76.yaml --params-file /tmp/domain76.yaml >/tmp/domain76.log 2>&1 &
pid=$!
trap 'kill -TERM "$pid" 2>/dev/null || true; tail -35 /tmp/domain76.log' EXIT
timeout 50 /usr/bin/python3 - <<'PY'
import math
import struct
import time
import rclpy
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
rclpy.init()
node=rclpy.create_node('elevation_domain76_monitor')
buf=Buffer()
listener=TransformListener(buf,node)
maps={}
pending=[]
pub=node.create_publisher(PointCloud2,"/elevation_orin_test76/timed_points",qos_profile_sensor_data)
last_sent=0
sent=0
exact=0
total=0
received=0
measured=[]
first_map=None
cloud_stats=[]
def cloud(msg):
    if not cloud_stats:
        pts=list(struct.iter_unpack("<ffff",msg.data))
        cloud_stats.append((msg.width,[(min(p[i] for p in pts),max(p[i] for p in pts)) for i in range(3)] if pts else []))
    pending.append((msg.header.frame_id,Time.from_msg(msg.header.stamp),time.monotonic(),msg))
def check(msg):
    global received, first_map
    received+=1
    if 'elevation' not in msg.layers:
        return
    count=sum(math.isfinite(x) for x in msg.data[msg.layers.index('elevation')].data)
    if count:
        assert list(msg.layers)==['elevation','min_filter','smooth']
        now=time.monotonic()
        if first_map is None:
            first_map=now
        if now-first_map>=2:
            measured.append((now,msg.header.stamp.sec,msg.header.stamp.nanosec))
        maps[(msg.header.stamp.sec,msg.header.stamp.nanosec)]=(msg.header.frame_id,count,msg.info.resolution,msg.info.length_x,msg.info.length_y)
subs=[node.create_subscription(GridMap,'/elevation_orin_test76/elevation_map',check,10),node.create_subscription(PointCloud2,'/elevation_orin_test76/livox_points',cloud,qos_profile_sensor_data)]
end=time.monotonic()+38
while time.monotonic()<end:
    rclpy.spin_once(node,timeout_sec=.1)
    now=time.monotonic()
    while pending and now-pending[0][2]>.8:
        frame,stamp,_,scan=pending.pop(0)
        total+=1
        try:
            buf.lookup_transform('odom',frame,stamp)
            exact+=1
            if now-last_sent>=0.19:
                pub.publish(scan)
                last_sent=now
                sent+=1
        except TransformException:
            pass
print('Exact scan TF coverage after 0.8 s:',exact,'/',total,flush=True)
print('Distinct nonempty maps:',len(maps),'latest:',next(reversed(maps.values()),None),flush=True)
print('Scans forwarded:',sent,'Maps received:',received,'cloud sample:',cloud_stats,'subs:',node.count_subscribers('/elevation_orin_test76/livox_points'),'map pubs:',node.count_publishers('/elevation_orin_test76/elevation_map'),flush=True)
print(node.get_subscriber_names_and_types_by_node('elevation_orin_test76','/'),flush=True)
if len(measured)>1:
    duration=measured[-1][0]-measured[0][0]
    print(f'Steady GridMap={(len(measured)-1)/duration:.2f} Hz, distinct scan timestamps={(len(set(x[1:] for x in measured))-1)/duration:.2f} Hz, window={duration:.1f} s',flush=True)
assert len(maps)>=3 and len(measured)>1
node.destroy_node()
rclpy.shutdown()
PY
CONTAINER
