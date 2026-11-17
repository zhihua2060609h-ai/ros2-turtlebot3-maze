"""按实验模式控制小车，统一发布速度指令并记录运行数据。"""
import csv
import json
import math
from pathlib import Path as FilePath
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from .control_math import sector, yaw, clip, point_command


class Controller(Node):
    def __init__(self):
        super().__init__('maze_controller')
        defaults = dict(mode='wall', adaptive=True, goal_x=1.0, goal_y=0.0,
                        waypoints='[[1,0],[1,1],[0,1],[0,0]]', target_left=0.34,
                        exit_x=8.55, exit_y_max=1000.0, max_duration=600.0,
                        log_dir=str(FilePath.home()/'ros2maze_ws'/'evidence'))
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.mode = self.get_parameter('mode').value
        self.adaptive = self.get_parameter('adaptive').value
        self.target = self.get_parameter('target_left').value
        self.goals = json.loads(self.get_parameter('waypoints').value)
        if self.mode == 'point':
            self.goals = [(self.get_parameter('goal_x').value, self.get_parameter('goal_y').value)]
        if self.mode not in ('wall','point','waypoints','open_loop','avoid','observe'):
            raise ValueError('Unknown mode: '+self.mode)
        self.index = 0
        self.pose = None
        self.scan = None
        self.last_odom = self.last_scan = 0.0
        self.start = None
        self.finished = False
        self.state = 'WAIT_SENSORS'
        self.last_state = ''
        self.travel = 0.0
        self.min_range = float('inf')
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        self.status_pub = self.create_publisher(String, '/maze_status', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        folder = FilePath(self.get_parameter('log_dir').value)
        folder.mkdir(parents=True, exist_ok=True)
        self.prefix = folder/(time.strftime('%Y%m%d_%H%M%S')+'_'+self.mode)
        self.file = open(str(self.prefix)+'.csv','w',newline='')
        self.csv = csv.writer(self.file)
        self.csv.writerow(['sim_s','elapsed_s','x_m','y_m','yaw_rad','front_m','left_m','right_m','v_mps','w_rps','state','waypoint'])
        self.tick = 0
        self.create_timer(0.05, self.control)
        self.get_logger().info('MODE='+self.mode+' CSV='+str(self.prefix)+'.csv')

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        if self.pose:
            delta = math.hypot(p.x-self.pose[0],p.y-self.pose[1])
            if delta < 0.5:
                self.travel += delta
        self.pose = (p.x,p.y,yaw(msg.pose.pose.orientation))
        self.last_odom = time.monotonic()
        if self.tick % 4 == 0:
            point = PoseStamped()
            point.header = msg.header
            point.pose = msg.pose.pose
            self.path.header.stamp = msg.header.stamp
            self.path.poses.append(point)
            self.path_pub.publish(self.path)

    def scan_cb(self, msg):
        self.scan = msg
        self.last_scan = time.monotonic()

    def finish(self, reason):
        if self.finished:
            return
        self.finished = True
        self.state = reason
        self.cmd.publish(Twist())
        result = dict(mode=self.mode, adaptive=self.adaptive, reason=reason,
                      elapsed_s=self.elapsed(), pose=self.pose, travel_m=self.travel,
                      min_sector_range_m=self.min_range if math.isfinite(self.min_range) else None,
                      waypoints_reached=self.index)
        FilePath(str(self.prefix)+'.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        self.file.flush()
        self.get_logger().info('RESULT '+json.dumps(result))

    def elapsed(self):
        return 0.0 if self.start is None else self.get_clock().now().nanoseconds/1e9-self.start

    def control(self):
        self.tick += 1
        now = self.get_clock().now().nanoseconds/1e9
        clock = time.monotonic()
        # 两路传感器数据均需及时更新，缺失或过期时发送零速度。
        fresh = self.pose is not None and self.scan is not None and clock-self.last_odom < 1.5 and clock-self.last_scan < 1.5
        if self.finished or not fresh:
            self.cmd.publish(Twist())
            if not self.finished:
                self.state='WAIT_SENSORS'
            return
        if self.start is None:
            self.start=now
        front=sector(self.scan,0,math.radians(20))
        left=sector(self.scan,math.pi/2,math.radians(12))
        right=sector(self.scan,-math.pi/2,math.radians(15))
        front_left=sector(self.scan,math.pi/4,math.radians(10))
        if any(v is None for v in (front,left,right,front_left)):
            self.cmd.publish(Twist())
            self.state='INVALID_SCAN'
            return
        self.min_range=min(self.min_range,front,left,right)
        elapsed=self.elapsed()
        if elapsed>self.get_parameter('max_duration').value:
            self.finish('TIMEOUT')
            return
        v=w=0.0
        if self.mode in ('point','waypoints'):
            v,w,reached,distance=point_command(*self.pose,self.goals[self.index])
            self.state='GOAL_'+str(self.index)
            if reached:
                self.index+=1
                self.get_logger().info('WAYPOINT_REACHED '+str(self.index))
                if self.index==len(self.goals):
                    self.finish('GOALS_REACHED')
                    return
            if front<0.22 and v>0:
                v=w=0.0
                self.state='FRONT_STOP'
        elif self.mode=='open_loop':
            if elapsed<3:
                v=0.15; self.state='STRAIGHT'
            elif elapsed<4:
                self.state='PAUSE'
            elif elapsed<7:
                w=0.6; self.state='ROTATE'
            else:
                self.finish('OPEN_LOOP_DONE'); return
        elif self.mode=='avoid':
            if front<0.5:
                w=-0.65; self.state='AVOID_RIGHT'
            else:
                v=0.14; self.state='FORWARD'
            if elapsed>15:
                self.finish('AVOID_DONE'); return
        elif self.mode=='observe':
            self.state='OBSERVE'
        else:
            # 出口坐标用于停止判定，沿墙方向由雷达测距决定。
            if (self.pose[0]>self.get_parameter('exit_x').value
                    and self.pose[1]<self.get_parameter('exit_y_max').value
                    and self.travel>3):
                self.finish('EXIT_REACHED'); return
            # 前方避让优先于寻找左墙，只有两个条件都不触发时才按墙距跟随。
            if front<0.38:
                w=-0.8; self.state='TURN_RIGHT'
            elif left>0.8:
                v=0.10; w=0.65; self.state='FIND_LEFT'
            else:
                error=left-self.target
                # 左前方过近时提前修正转向，前方越近或转向越急时降低线速度。
                w=clip(2.2*error-1.0*max(0,0.42-front_left),0.65)
                v=0.17 if not self.adaptive else max(0.065,0.18*(1-0.65*abs(w)/0.65))*min(1.0,max(0.35,front/0.8))
                self.state='FOLLOW_LEFT'
        twist=Twist(); twist.linear.x=float(v); twist.angular.z=float(w)
        self.cmd.publish(twist)
        if self.state!=self.last_state:
            self.get_logger().info('STATE '+self.state)
            self.last_state=self.state
        if self.tick%4==0:
            self.status_pub.publish(String(data=json.dumps(dict(mode=self.mode,state=self.state,x=self.pose[0],y=self.pose[1],front=front,left=left,right=right))))
            self.csv.writerow([now,elapsed,*self.pose,front,left,right,v,w,self.state,self.index])
            self.file.flush()


def main(args=None):
    rclpy.init(args=args)
    node=Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())
        node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
