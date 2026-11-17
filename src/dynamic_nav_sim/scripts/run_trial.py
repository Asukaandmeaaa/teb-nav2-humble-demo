#!/usr/bin/env python3
"""Compare actual straight-driving collision with Nav2/TEB on the same scenario."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from gazebo_msgs.msg import ModelStates, ContactsState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as NavigationPath
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock as ClockMessage
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
from ament_index_python.packages import get_package_share_directory


def polygon_distance(a,b):
    # SAT detects touching/overlap; segment distance gives separation otherwise.
    separated=False
    for polygon in (a,b):
        for i,p in enumerate(polygon):
            q=polygon[(i+1)%len(polygon)];nx,ny=-(q[1]-p[1]),q[0]-p[0]
            pa=[x*nx+y*ny for x,y in a];pb=[x*nx+y*ny for x,y in b]
            if max(pa)<min(pb) or max(pb)<min(pa):separated=True
    if not separated:return 0.0
    def segment(p,a,b):
        dx,dy=b[0]-a[0],b[1]-a[1]
        t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
        return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
    return min(segment(p,poly[i],poly[(i+1)%len(poly)]) for pts,poly in ((a,b),(b,a)) for p in pts for i in range(len(poly)))


def percentile(values,q):
    if not values:return None
    values=sorted(values);index=(len(values)-1)*q
    lo=int(index);hi=min(lo+1,len(values)-1)
    return values[lo]+(values[hi]-values[lo])*(index-lo)


def scenario_freshness(scene,scenario,rows):
    """Keep benchmark start checks separate from ROS communication readiness."""
    start=scene.get('start')
    position=list(rows[-1][1:3]) if rows else None
    distance=math.dist(position,start[:2]) if position is not None and start else None
    reasons=[]
    if scenario.get('phase')!='ARMED':
        reasons.append('场景阶段不是 ARMED，障碍可能已经触发')
    if distance is None:
        reasons.append('尚无起点或小车位置数据')
    elif distance>.15:
        reasons.append(f'小车距起点 {distance:.3f} m，超过允许的 0.150 m；可能已行驶或长时间静置后发生物理漂移')
    return {'fresh':not reasons,'reasons':reasons,'phase':scenario.get('phase'),
            'expected_start_xy':start,'robot_xy':position,
            'start_offset_m':distance,'max_start_offset_m':.15}


class Trial(Node):
    def __init__(self):
        super().__init__('dynamic_avoidance_trial')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.active_nodes=set()
        self.lifecycle_pending={}
        self.lifecycle_clients={name:self.create_client(GetState,'/'+name+'/get_state') for name in ('bt_navigator','controller_server','planner_server','velocity_smoother')}
        # Diagnostics must keep running when Gazebo's simulation clock is paused.
        self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.5,self.check_lifecycle,clock=self.wall_clock)
        self.action=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.baseline_pub=self.create_publisher(Twist,'/cmd_vel_nav',10)
        self.baseline_enabled=False
        self.create_timer(.05,self.drive_straight)
        self.rows=[];self.scans=0;self.commands=0
        self.raw_states=0;self.raw_scans=0;self.clock_messages=0
        self.online_map_messages=0;self.local_grid_samples=[]
        self.clock_stamp=None;self.clock_last_advance=None
        self.stage='waiting';self.started_wall=time.monotonic()
        self.min_clearance=float('inf')
        self.contacts=[];self.scenario={};self.phases=set()
        self.scene_geometry={};self.static_polygons=[];self.min_static_clearance=float("inf")
        self.compute_metrics=[];self.raw_command_times=[]
        self.obstacle_stats={};self.obstacle_phases={}
        self.last_command=(0.,0.)
        self.plan_samples={'global':[], 'local':[]}
        manifest=json.loads((Path(get_package_share_directory('dynamic_nav_sim'))/'config/robot_geometry.json').read_text())
        points=manifest['footprint_with_2cm_margin']
        self.xmin=min(p[0] for p in points);self.xmax=max(p[0] for p in points)
        self.ymin=min(p[1] for p in points);self.ymax=max(p[1] for p in points)
        self.create_subscription(ModelStates,'/gazebo/model_states',self.states,qos_profile_sensor_data)
        self.create_subscription(ContactsState,'/obstacle_contacts',self.contact,qos_profile_sensor_data)
        self.create_subscription(ContactsState,'/static_contacts',self.contact,qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray,'/teb_compute_metrics',lambda msg:self.compute_metrics.append(list(msg.data)),100)
        self.create_subscription(Twist,'/cmd_vel_nav',lambda msg:self.raw_command_times.append(time.monotonic()),100)
        self.create_subscription(String,'/scenario/state',self.scenario_state,10)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.create_subscription(Twist,'/cmd_vel',self.command,10)
        self.create_subscription(ClockMessage,'/clock',self.clock_received,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',self.online_map,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/local_costmap/costmap',self.local_grid,qos_profile_sensor_data)
        self.create_subscription(NavigationPath,'/plan',lambda msg:self.record_plan('global',msg),10)
        self.create_subscription(NavigationPath,'/local_plan',lambda msg:self.record_plan('local',msg),10)
        self.create_timer(5.,self.progress,clock=self.wall_clock)

    def record_plan(self,kind,msg):
        # Visualization only: recorded planner output never feeds navigation.
        now=self.get_clock().now().nanoseconds*1.e-9
        samples=self.plan_samples[kind]
        if samples and now-samples[-1]['sim_time']<.5:return
        points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]
        stride=max(1,math.ceil(len(points)/250))
        reduced=points[::stride]
        if points and reduced[-1]!=points[-1]:reduced.append(points[-1])
        samples.append({'sim_time':now,'frame':msg.header.frame_id,'points':reduced})

    def online_map(self,msg):
        if msg.info.width and msg.info.height:self.online_map_messages+=1

    def local_grid(self,msg):
        info=msg.info
        self.local_grid_samples.append({'sim_time':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            'frame':msg.header.frame_id,'width':info.width,'height':info.height,'resolution':info.resolution,
            'origin_x':info.origin.position.x,'origin_y':info.origin.position.y})

    def clock_received(self,msg):
        stamp=msg.clock.sec+msg.clock.nanosec*1e-9
        self.clock_messages+=1
        if self.clock_stamp is not None and stamp>self.clock_stamp:
            self.clock_last_advance=time.monotonic()
        self.clock_stamp=stamp

    def readiness_missing(self):
        missing=[]
        if self.clock_last_advance is None or time.monotonic()-self.clock_last_advance>2.:
            missing.append('/clock 未推进（仿真可能暂停或未正常运行）')
        if not self.action.server_is_ready():missing.append('/navigate_to_pose action')
        missing.extend(name+' 未激活/未响应' for name in self.lifecycle_clients if name not in self.active_nodes)
        if self.scans<=5:missing.append(f'/scan 有效消息 {self.scans}/6')
        if not self.online_map_messages:missing.append('/map 在线地图尚未收到')
        if not self.local_grid_samples:missing.append('/local_costmap/costmap 尚未收到')
        if not self.scenario:missing.append('/scenario/state')
        if not self.scene_geometry:missing.append('场景几何')
        if len(self.rows)<=5:missing.append(f'有效模型状态 {len(self.rows)}/6（原始收到 {self.raw_states}）')
        if self.scene_geometry:
            for topic,key in (('/obstacle_contacts','dynamic_obstacles'),('/static_contacts','boxes')):
                actual=self.count_publishers(topic);expected=len(self.scene_geometry[key])
                if key=='boxes':expected+=len(self.scene_geometry.get('oriented_boxes',[]))
                if actual<expected:missing.append(f'{topic} 发布者 {actual}/{expected}')
        return missing

    def diagnostics(self):
        return {
            'missing':self.readiness_missing(),
            'ros_domain_id':os.environ.get('ROS_DOMAIN_ID','0'),
            'clock_messages':self.clock_messages,'last_clock_seconds':self.clock_stamp,
            'seconds_since_clock_advanced':time.monotonic()-self.clock_last_advance if self.clock_last_advance is not None else None,
            'raw_scan_messages':self.raw_scans,'valid_scan_messages':self.scans,
            'online_map_messages':self.online_map_messages,
            'local_grid':self.local_grid_samples[-1] if self.local_grid_samples else None,
            'raw_model_state_messages':self.raw_states,'state_samples':len(self.rows),
            'active_nodes':sorted(self.active_nodes),
            'publishers':{topic:self.count_publishers(topic) for topic in ('/clock','/scan','/map','/gazebo/model_states','/scenario/state')},
        }

    def progress(self):
        elapsed=time.monotonic()-self.started_wall
        if self.stage=='waiting':
            self.get_logger().info(f'等待就绪 {elapsed:.0f}s: '+('; '.join(self.readiness_missing()) or '已就绪'))
        elif self.stage=='navigating' and self.rows:
            row=self.rows[-1]
            self.get_logger().info(f'导航中 {elapsed:.0f}s: 位置=({row[1]:.2f}, {row[2]:.2f}), 场景阶段={self.scenario.get("phase")}, 接触次数={len(self.contacts)}')

    def check_lifecycle(self):
        for name,client in self.lifecycle_clients.items():
            future=self.lifecycle_pending.get(name)
            if future is not None:
                if not future.done():continue
                try:
                    if future.result().current_state.id==State.PRIMARY_STATE_ACTIVE:self.active_nodes.add(name)
                    else:self.active_nodes.discard(name)
                except Exception:self.active_nodes.discard(name)
            if client.service_is_ready():self.lifecycle_pending[name]=client.call_async(GetState.Request())

    def scenario_state(self,msg):
        self.scenario=json.loads(msg.data)
        self.phases.add(self.scenario['phase'])
        for item in self.scenario.get('obstacles',[]):
            self.obstacle_phases.setdefault(item['name'],set()).add(item['phase'])
        if not self.scene_geometry:
            self.scene_geometry=json.loads((Path(get_package_share_directory('dynamic_nav_sim'))/'config'/(self.scenario.get('scene','dynamic_crossing')+'_scene.json')).read_text())
            self.static_polygons=[item['points'] for item in self.scene_geometry.get('static_polygons',[])]
            if not self.static_polygons:
                self.static_polygons=[[(x-sx/2,y-sy/2),(x+sx/2,y-sy/2),(x+sx/2,y+sy/2),(x-sx/2,y+sy/2)] for _,x,y,sx,sy in self.scene_geometry.get('boxes',[])]

    def drive_straight(self):
        if self.baseline_enabled:
            msg=Twist();msg.linear.x=.45
            self.baseline_pub.publish(msg)

    def contact(self,msg):
        for state in msg.states:
            if 'demo_cart::' in state.collision1_name or 'demo_cart::' in state.collision2_name:
                if not self.contacts:
                    self.get_logger().error('ACTUAL GAZEBO CONTACT: '+state.collision1_name+' / '+state.collision2_name)
                self.contacts.append({'sim_time':self.get_clock().now().nanoseconds*1e-9,'collision1':state.collision1_name,'collision2':state.collision2_name})

    def scan(self,msg):
        self.raw_scans+=1
        if any(math.isfinite(r) and msg.range_min<=r<=msg.range_max for r in msg.ranges):self.scans+=1

    def command(self,msg):
        self.last_command=(msg.linear.x,msg.angular.z)
        if abs(msg.linear.x)+abs(msg.angular.z)>.001:self.commands+=1

    def states(self,msg):
        self.raw_states+=1
        now=self.get_clock().now().nanoseconds*1e-9
        if now==0. or 'demo_cart' not in msg.name or not self.scene_geometry:return
        specs=self.scene_geometry['dynamic_obstacles']
        if any(spec['name'] not in msg.name for spec in specs):return
        idx=msg.name.index('demo_cart');p=msg.pose[idx];v=msg.twist[idx]
        q=p.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        measured=[]
        for spec in specs:
            o=msg.pose[msg.name.index(spec['name'])]
            dx,dy=o.position.x-p.position.x,o.position.y-p.position.y
            bx=math.cos(yaw)*dx+math.sin(yaw)*dy;by=-math.sin(yaw)*dx+math.cos(yaw)*dy
            gap=math.hypot(max(self.xmin-bx,0.,bx-self.xmax),max(self.ymin-by,0.,by-self.ymax))-spec['radius']
            measured.append([o.position.x,o.position.y,gap])
            stats=self.obstacle_stats.setdefault(spec['name'],{'min_clearance_m':gap,'min_y':o.position.y,'max_y':o.position.y})
            stats['min_clearance_m']=min(stats['min_clearance_m'],gap)
            stats['min_y']=min(stats['min_y'],o.position.y);stats['max_y']=max(stats['max_y'],o.position.y)
        clearance=min(item[2] for item in measured)
        self.min_clearance=min(self.min_clearance,clearance)
        forward_speed=math.cos(yaw)*v.linear.x+math.sin(yaw)*v.linear.y
        footprint=[(p.position.x+math.cos(yaw)*x-math.sin(yaw)*y,p.position.y+math.sin(yaw)*x+math.cos(yaw)*y) for x,y in [(self.xmin,self.ymin),(self.xmax,self.ymin),(self.xmax,self.ymax),(self.xmin,self.ymax)]]
        static_clearance=min((polygon_distance(footprint,polygon) for polygon in self.static_polygons),default=float('inf'))
        self.min_static_clearance=min(self.min_static_clearance,static_clearance)
        self.rows.append([now,p.position.x,p.position.y,yaw,measured[0][0],measured[0][1],clearance,forward_speed,self.last_command[0],self.last_command[1],static_clearance]+[value for item in measured[1:] for value in item])


def spin_until(node,predicate,timeout):
    end=time.monotonic()+timeout
    while rclpy.ok() and not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    return predicate()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mode',choices=['teb','straight'],default='teb')
    parser.add_argument('--timeout',type=float,default=None,help='Default: scene recommended timeout')
    parser.add_argument('--x',type=float,default=None,help='Default: scene goal x')
    parser.add_argument('--y',type=float,default=None,help='Default: scene goal y')
    parser.add_argument('--output',default='/workspace/results')
    parser.add_argument('--ready-timeout',type=float,default=60.,help='Wall seconds to wait for live simulation data and Nav2')
    parser.add_argument('--check-only',action='store_true',help='Check readiness without sending a navigation goal')
    parser.add_argument('--verbose-report',action='store_true',help='Print the complete JSON report (always saved to disk)')
    args=parser.parse_args()
    if args.ready_timeout<=0:parser.error('--ready-timeout must be positive')
    if args.mode=='straight' and args.y not in (None,0.):parser.error('straight baseline requires y=0')
    rclpy.init(args=[]);node=Trial();status='NOT_READY';handle=None;passed=False
    started=time.monotonic()
    def goal_distance():
        return math.hypot(node.rows[-1][1]-args.x,node.rows[-1][2]-args.y) if node.rows else float('inf')
    try:
        node.get_logger().info(f'已进入容器，等待仿真数据和 Nav2 就绪（最多 {args.ready_timeout:g}s）。')
        ready=spin_until(node,lambda:not node.readiness_missing(),args.ready_timeout)
        readiness=node.diagnostics()
        freshness=scenario_freshness(node.scene_geometry,node.scenario,node.rows)
        node.stage='idle'
        if not ready:
            node.get_logger().error('未发送导航目标，缺少：'+'; '.join(readiness['missing']))
            if node.clock_last_advance is None or time.monotonic()-node.clock_last_advance>2.:
                node.get_logger().error('请先执行 ./scripts/sim.sh resume 恢复物理仿真，再执行 status 检查；若仍无数据，查看 ./scripts/sim.sh logs。')
        if ready and not freshness['fresh']:
            node.get_logger().error('SCENARIO_NOT_FRESH：未发送导航目标。'+'；'.join(freshness['reasons']))
            node.get_logger().error('在终端 1 执行 ./scripts/sim.sh restart gui 重建场景和在线地图；随后在终端 2 执行 ./scripts/sim.sh trial。继续等待或 resume 不会重置起点。')
        if args.check_only:
            print(json.dumps({'ready':ready,'trial_ready':ready and freshness['fresh'],
                              'freshness':freshness,'diagnostics':readiness},ensure_ascii=False,indent=2),flush=True)
            return 0 if ready and freshness['fresh'] else 1
        goal_xy=node.scene_geometry.get('goal',[None,None])
        args.x=goal_xy[0] if args.x is None else args.x
        args.y=goal_xy[1] if args.y is None else args.y
        args.timeout=node.scene_geometry.get('recommended_timeout',90.) if args.timeout is None else args.timeout
        if ready and not freshness['fresh']:
            status='SCENARIO_NOT_FRESH'
        elif ready and args.mode=='straight':
            node.stage='navigating'
            node.baseline_enabled=True
            spin_until(node,lambda:bool(node.contacts) or goal_distance()<.2,args.timeout)
            status='COLLISION' if node.contacts else ('BASELINE_GOAL_REACHED' if goal_distance()<.2 else 'TIMEOUT')
            node.baseline_enabled=False
            for _ in range(20):
                node.baseline_pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.05)
        elif ready:
            node.stage='navigating'
            node.get_logger().info(f'已就绪，发送导航目标 ({args.x:.2f}, {args.y:.2f})。')
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='odom'
            # Resolve the odometry-frame target using the latest SLAM transform.
            # A fixed old timestamp expires during long navigation/replanning.
            goal.pose.pose.position.x=args.x;goal.pose.pose.position.y=args.y;goal.pose.pose.orientation.w=1.
            future=node.action.send_goal_async(goal)
            if spin_until(node,future.done,10.):
                handle=future.result()
                if handle.accepted:
                    result=handle.get_result_async()
                    spin_until(node,lambda:result.done() or bool(node.contacts),args.timeout)
                    if node.contacts:status='COLLISION'
                    elif result.done():status={GoalStatus.STATUS_SUCCEEDED:'SUCCEEDED',GoalStatus.STATUS_ABORTED:'ABORTED',GoalStatus.STATUS_CANCELED:'CANCELED'}.get(result.result().status,str(result.result().status))
                    else:status='TIMEOUT'
                    if not result.done():
                        cancel=handle.cancel_goal_async();spin_until(node,cancel.done,5.)
                else:status='REJECTED'
            else:status='GOAL_RESPONSE_TIMEOUT'
        node.stage='idle'
        end_error=goal_distance() if node.rows and args.x is not None and args.y is not None else None
        max_lateral=max(abs(row[2]) for row in node.rows) if node.rows else 0.
        span=max(r[5] for r in node.rows)-min(r[5] for r in node.rows) if node.rows else 0.
        dynamic_details={item['name']:{**node.obstacle_stats.get(item['name'],{}),'trigger_time':item['trigger_time'],'phase':item['phase'],'phases':sorted(node.obstacle_phases.get(item['name'],set()))} for item in node.scenario.get('obstacles',[])}
        expected=[item for item in node.scene_geometry.get('dynamic_obstacles',[]) if item['enabled']]
        triggered=bool(expected) and all(dynamic_details.get(item['name'],{}).get('trigger_time') is not None and dynamic_details[item['name']].get('max_y',0)-dynamic_details[item['name']].get('min_y',0)>=abs(item['start_y']) for item in expected)
        passed=status=='SUCCEEDED' and not node.contacts and node.min_clearance>0 and node.scans>0 and node.commands>0 and (triggered or not node.scenario.get('dynamic_enabled',True)) and node.min_static_clearance>0 and end_error is not None and end_error<.35
        report={'scene_geometry':node.scene_geometry,'dynamic_obstacle_results':dynamic_details,'all_dynamic_obstacles_exercised':triggered,'passed':passed,'mode':args.mode,'action_status':status,'wall_seconds':time.monotonic()-started,'sampled_sim_seconds':node.rows[-1][0]-node.rows[0][0] if node.rows else 0.,'goal':[args.x,args.y],'final_goal_error_m':end_error,'min_padded_footprint_obstacle_clearance_m':node.min_clearance if node.rows else None,'max_lateral_deviation_m':max_lateral,'physical_contact_count':len(node.contacts),'first_contact':node.contacts[0] if node.contacts else None,'scenario':node.scenario,'scenario_phases':sorted(node.phases),'obstacle_y_span_m':span,'scan_messages':node.scans,'nonzero_velocity_commands':node.commands,'state_samples':len(node.rows),'measurement_note':'Contact sensors on every cylinder and static box; padded polygon clearance sampled at model-state rate. Compute timing is wall duration inside TEB, not end-to-end perception latency.'}
        durations=[row[0] for row in node.compute_metrics]
        interval=[b-a for a,b in zip(node.raw_command_times,node.raw_command_times[1:])]
        budget=50. if node.scenario.get('profile')=='responsive' else 100.
        moving=next((i for i,row in enumerate(node.rows) if abs(row[7])>.03),len(node.rows))
        pairs=list(zip(node.rows[moving:],node.rows[moving+1:]))
        report.update({
            'readiness_diagnostics':readiness,
            'start_freshness':freshness,
            'plan_samples':node.plan_samples,
            'online_map_messages':node.online_map_messages,
            'local_grid_samples':node.local_grid_samples,
            'min_padded_static_clearance_m':node.min_static_clearance if math.isfinite(node.min_static_clearance) else None,
            'path_length_m':sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in pairs),
            'stopped_sim_seconds':sum(b[0]-a[0] for a,b in pairs if abs(a[7])<.03),
            'reverse_sim_seconds':sum(b[0]-a[0] for a,b in pairs if a[7]<-.03),
            'compute_calls':len(durations),'compute_p50_ms':percentile(durations,.5),
            'compute_p95_ms':percentile(durations,.95),'compute_max_ms':max(durations,default=None),
            'compute_failed_calls':sum(row[1]<.5 for row in node.compute_metrics),
            'compute_over_budget_calls':sum(t>budget for t in durations),'compute_budget_ms':budget,
            'raw_command_interval_wall_p95_ms':1000*percentile(interval,.95) if interval else None,
            'gates_crossed':sum(any(abs(row[1]-gate['x'])<.08 and abs(row[2]-gate['y'])<gate['width']/2 for row in node.rows) for gate in node.scene_geometry.get('gates',[])),
        })
        passed=passed and report['gates_crossed']==len(node.scene_geometry.get('gates',[])) and report['compute_calls']>0
        report['passed']=passed
        output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
        name=time.strftime('trial_%Y%m%d_%H%M%S',time.gmtime())+'_'+args.mode+'_UTC'
        (output/(name+'.json')).write_text(json.dumps(report,indent=2)+'\n')
        with (output/(name+'.csv')).open('w') as f:
            writer=csv.writer(f);writer.writerow(['sim_time','robot_x','robot_y','robot_yaw','obstacle_x','obstacle_y','clearance_m','measured_forward_speed','cmd_v','cmd_w','static_clearance_m']+[f'obstacle_{i}_{field}' for i in range(2,len(node.scene_geometry.get('dynamic_obstacles',[]))+1) for field in ('x','y','clearance_m')]);writer.writerows(node.rows)
        summary_keys=('action_status','passed','wall_seconds','final_goal_error_m',
                      'physical_contact_count','gates_crossed','all_dynamic_obstacles_exercised',
                      'min_padded_static_clearance_m','min_padded_footprint_obstacle_clearance_m',
                      'stopped_sim_seconds','compute_p95_ms','compute_max_ms','start_freshness')
        print(json.dumps(report if args.verbose_report else {key:report[key] for key in summary_keys},
                         ensure_ascii=False,indent=2))
        print('Report: '+str(output/(name+'.json')))
    finally:
        node.baseline_enabled=False
        if args.mode=='straight' and not args.check_only:node.baseline_pub.publish(Twist())
        node.destroy_node();rclpy.shutdown()
    return 0 if passed else 1

if __name__=='__main__':raise SystemExit(main())
