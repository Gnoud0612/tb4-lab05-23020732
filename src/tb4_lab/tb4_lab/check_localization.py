"""Observe a restarted maze simulation; never publish commands or pose estimates."""
import argparse
import json
import math
from pathlib import Path
import time
from datetime import datetime, timezone
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformException


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--duration',type=float,default=12)
    p.add_argument('--output',default='evidence/localization.json')
    a=p.parse_args()
    if a.duration<=0:p.error('duration must be positive')
    rclpy.init()
    n=Node('tb4_lab_localization_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
    b=Buffer();listener=TransformListener(b,n)
    state={};samples=[]
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def truth(m):
        v=m.pose.pose
        state['ground_truth']={'x':v.position.x,'y':v.position.y,'yaw':yaw(v.orientation),'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9}
    def amcl(m):
        v=m.pose.pose
        state['amcl']={'x':v.position.x,'y':v.position.y,'yaw':yaw(v.orientation),'covariance_x':m.pose.covariance[0],'covariance_y':m.pose.covariance[7],'covariance_yaw':m.pose.covariance[35]}
    def map_msg(m):
        state['map']={'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution,'frame':m.header.frame_id}
    n.create_subscription(Odometry,'/sim_ground_truth_pose',truth,qos_profile_sensor_data)
    n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',amcl,latched)
    n.create_subscription(OccupancyGrid,'/map',map_msg,latched)
    last_stamp=None
    end=time.monotonic()+a.duration
    try:
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.1)
            if 'ground_truth' not in state:continue
            if state['ground_truth']['stamp']==last_stamp:continue
            try:t=b.lookup_transform('map','base_link',rclpy.time.Time())
            except TransformException:continue
            stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
            if abs(stamp-state['ground_truth']['stamp'])>.2:continue
            x=t.transform.translation.x;y=t.transform.translation.y;ang=yaw(t.transform.rotation)
            g=state['ground_truth']
            last_stamp=g['stamp']
            samples.append({'map_pose':{'x':x,'y':y,'yaw':ang},'ground_truth':dict(g),'position_difference_m':math.hypot(x-g['x'],y-g['y']),'yaw_difference_rad':abs(math.atan2(math.sin(ang-g['yaw']),math.cos(ang-g['yaw'])))})
        names=sorted(name for name,ns in n.get_node_names_and_namespaces())
        good=[s for s in samples if s['position_difference_m']<=.4 and s['yaw_difference_rad']<=.3]
        passed=bool(len(samples)>=10 and len(good)/len(samples)>=.9 and 'amcl' in state and 'map' in state and 'slam_toolbox' not in names)
        result={'created_utc':datetime.now(timezone.utc).isoformat(),'passed':passed,'assumption':'Same official maze, simulator spawn (0,0,0); SLAM map uses initial odometry frame. Differences include any SLAM map/world alignment error.','sample_count':len(samples),'acceptable_sample_count':len(good),'position_tolerance_m':.4,'yaw_tolerance_rad':.3,'latest':samples[-1] if samples else None,'received':state,'nodes':names}
        out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n')
        print(('PASS' if passed else 'FAIL')+': '+str(out))
    finally:n.destroy_node();rclpy.try_shutdown()
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
