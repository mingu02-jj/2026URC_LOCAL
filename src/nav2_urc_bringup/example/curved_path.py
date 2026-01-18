#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


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
        super().__init__("follow_path_client")
        self.cli = ActionClient(self, FollowPath, "/follow_path")

        # ✅ RViz 시각화용 퍼블리셔
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.viz_pub = self.create_publisher(Path, "curved_path", qos)

    def send(self, path: Path):
        # ✅ RViz 표시용 publish
        self.viz_pub.publish(path)

        self.get_logger().info("Waiting for /follow_path action server...")
        self.cli.wait_for_server()

        goal = FollowPath.Goal()
        goal.path = path

        self.get_logger().info(f"Sending path with {len(path.poses)} poses")
        send_future = self.cli.send_goal_async(goal, feedback_callback=self._fb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f"Result: {result_future.result().result}")

    def _fb(self, feedback_msg):
        fb = feedback_msg.feedback
        if hasattr(fb, "current_pose"):
            p = fb.current_pose.pose.position
            self.get_logger().info(f"feedback: current=({p.x:.2f},{p.y:.2f})")
        else:
            self.get_logger().info("feedback received")


def main():
    rclpy.init()
    node = FollowPathClient()

    frame = "odom"
    now = node.get_clock().now().to_msg()

    # ✅ 경로 설정 (0, 0) → (10, 0)
    N = 200  # 점 개수 (곡선 부드러움)
    start_x, start_y = 0.0, 0.0
    goal_x, goal_y = 10.0, 0.0

    amplitude = 1.0               # y축 휨 정도 (1m)
    frequency = 2 * math.pi / 5.0  # 5m당 1회 S자 파형

    pts = []
    for i in range(N):
        t = i / (N - 1)
        x = start_x + (goal_x - start_x) * t
        y = amplitude * math.sin(frequency * x)  # S자 곡선
        yaw = math.atan2(
            amplitude * frequency * math.cos(frequency * x), 1.0
        )  # 진행방향 따라 회전
        pts.append((x, y, yaw))

    path = Path()
    path.header.frame_id = frame
    path.header.stamp = now
    path.poses = [make_pose(x, y, yaw, frame, now) for (x, y, yaw) in pts]

    node.send(path)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
