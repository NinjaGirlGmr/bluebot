 #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from hardware_interface.system_interface import SystemInterface

class SerialDiffDriveHW(SystemInterface):
    def __init__(self):
        super().__init__()
        self.joint_names = ["left_wheel_joint", "right_wheel_joint"]
        self.position = [0.0, 0.0]
        self.velocity = [0.0, 0.0]
        self.cmd_velocity = [0.0, 0.0]

        self.node = Node('serial_diff_drive_hw')

        # Subscribe to your serial bridge joint states
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publish cmd_vel to your serial bridge
        self.cmd_pub = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.position[i] = msg.position[idx]
                self.velocity[i] = msg.velocity[idx]

    def read(self):
        # ros2_control calls this
        return True

    def write(self):
        # Convert wheel velocities to Twist and publish
        twist = Twist()
        twist.linear.x = (self.cmd_velocity[0] + self.cmd_velocity[1]) / 2.0
        twist.angular.z = (self.cmd_velocity[1] - self.cmd_velocity[0]) / 0.195  # track width
        self.cmd_pub.publish(twist)
        return True

def main(args=None):
    rclpy.init(args=args)
    hw = SerialDiffDriveHW()
    rclpy.spin(hw.node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
