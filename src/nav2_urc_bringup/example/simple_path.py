#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import FollowPath


def yaw_to_quat(yaw: float) -> Quaternion:
    """단순 2D yaw(라디안)를 quaternion(z, w)으로 변환."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def quat_to_yaw(q: Quaternion) -> float:
    """
    quaternion → yaw(라디안) 변환.
    전체 3D 쿼터니언 공식 중 z축 yaw 성분만 계산.
    """
    # sin(yaw), cos(yaw) 조합
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_pose(x, y, yaw, frame, stamp):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = stamp
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = 0.0
    ps.pose.orientation = yaw_to_quat(yaw)
    return ps


class FollowPathClient(Node):
    def __init__(self):
        super().__init__('follow_path_client')
        self.cli = ActionClient(self, FollowPath, '/follow_path')

        # 현재 odom pose 저장용
        self.current_pose = None
        self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',          # 필요시 /amcl_pose 등으로 변경 가능
            self._odom_cb,
            10
        )

        # RViz 시각화용 퍼블리셔 (지속 QoS)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.viz_pub = self.create_publisher(Path, 'linear_path', qos)

        # dry_run 파라미터 (기본 False)
        self.declare_parameter('dry_run', False)

    # ===== odom 콜백 =====
    def _odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    # ===== 현재 pose 기준 직선 path 생성 =====
    def make_straight_path_from_current_pose(self,
                                             length: float = 10.0,
                                             step: float = 0.1,
                                             frame: str = 'odom') -> Path | None:
        if self.current_pose is None:
            self.get_logger().warn('No /odom received yet, cannot build path.')
            return None

        x0 = self.current_pose.position.x
        y0 = self.current_pose.position.y
        q = self.current_pose.orientation
        yaw0 = quat_to_yaw(q)

        N = int(length / step)
        now = self.get_clock().now().to_msg()

        path = Path()
        path.header.frame_id = frame
        path.header.stamp = now

        poses = []
        for i in range(N):
            s = step * i
            x = x0 + s * math.cos(yaw0)
            y = y0 + s * math.sin(yaw0)
            poses.append(make_pose(x, y, yaw0, frame, now))

        path.poses = poses

        self.get_logger().info(
            f'Built straight path from current pose: '
            f'start=({x0:.2f}, {y0:.2f}, yaw={yaw0:.2f}), N={N}'
        )
        return path

    # ===== FollowPath 전송 =====
    def send(self, path: Path):
        # 1) RViz용 Path publish
        self.viz_pub.publish(path)
        self.get_logger().info(
            f'Published path with {len(path.poses)} poses to /linear_path'
        )

        # RViz가 subscribe 할 시간을 잠깐 줌
        rclpy.spin_once(self, timeout_sec=0.1)

        # 2) dry_run이면 여기서 일정 시간 동안만 살아 있다가 종료
        dry_run = self.get_parameter('dry_run').get_parameter_value().bool_value
        if dry_run:
            self.get_logger().info(
                'Dry-run mode: NOT sending FollowPath action. '
                'Waiting a few seconds so RViz can connect...'
            )
            for _ in range(1000):  # 0.1s * 100 = 10초
                if not rclpy.ok():
                    break
                rclpy.spin_once(self, timeout_sec=0.1)
            return

        # 3) 실제 로봇 움직이게 하는 FollowPath 액션 전송
        self.get_logger().info('Waiting for /follow_path action server...')
        self.cli.wait_for_server()

        goal = FollowPath.Goal()
        goal.path = path

        self.get_logger().info(f'Sending path with {len(path.poses)} poses')
        send_future = self.cli.send_goal_async(goal, feedback_callback=self._fb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'Result: {result_future.result().result}')

    def _fb(self, feedback_msg):
        fb = feedback_msg.feedback
        if hasattr(fb, 'current_pose'):
            p = fb.current_pose.pose.position
            self.get_logger().info(f'feedback: current=({p.x:.2f},{p.y:.2f})')
        else:
            self.get_logger().info('feedback received')


def main():
    rclpy.init()
    node = FollowPathClient()

    # --- /odom 올 때까지 잠깐 대기 ---
    while rclpy.ok() and node.current_pose is None:
        node.get_logger().info('Waiting for /odom...')
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.current_pose is None:
        node.get_logger().error('No /odom received. Shutting down.')
        node.destroy_node()
        rclpy.shutdown()
        return

    # --- 현재 pose 기준으로 앞으로 10m 직선 path 생성 ---
    path = node.make_straight_path_from_current_pose(
        length=7.0,  # 10 m 직선
        step=0.1,     # 0.1 m 간격
        frame='odom'
    )
    if path is not None:
        node.send(path)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

