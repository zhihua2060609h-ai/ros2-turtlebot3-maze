"""雷达分区、姿态角与点到点控制的计算函数。"""
import math

def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def clip(value, limit):
    return max(-limit, min(limit, value))

def sector(scan, centre, half_width):
    """按扫描起始角和角增量确定扇区，返回其中的最近可用距离。"""
    values = []
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        if abs(wrap(angle - centre)) > half_width:
            continue
        # 正无穷按最大量程处理。扇区没有可用读数时返回 None，交由控制节点停车。
        if math.isinf(distance) and distance > 0:
            values.append(scan.range_max)
        elif math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
            values.append(distance)
    return min(values) if values else None

def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def point_command(x, y, heading, goal, tolerance=0.05):
    distance = math.hypot(goal[0]-x, goal[1]-y)
    error = wrap(math.atan2(goal[1]-y, goal[0]-x)-heading)
    if distance < tolerance:
        return 0.0, 0.0, True, distance
    # 航向误差过大时先转向，允许前进后按距离误差调速。
    v = min(0.18, 0.65*distance)*max(0.0, math.cos(error)) if abs(error)<0.6 else 0.0
    return v, clip(1.8*error, 1.0), False, distance
