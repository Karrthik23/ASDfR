import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import time

class TrackingPlotter(Node):
    def __init__(self):
        super().__init__('tracking_plotter')
        self.setpoint_right = []
        self.setpoint_left = []
        self.actual_position = []
        self.time_data = []
        self.start_time = time.time()

        self.create_subscription(Float64, '/input/right_motor/setpoint_vel', self.right_setpoint_callback, 10)
        self.create_subscription(Float64, '/input/left_motor/setpoint_vel', self.left_setpoint_callback, 10)
        self.create_subscription(Point, '/output/robot_pose', self.position_callback, 10)

    def right_setpoint_callback(self, msg):
        self.setpoint_right.append(msg.data)

    def left_setpoint_callback(self, msg):
        self.setpoint_left.append(msg.data)

    def position_callback(self, msg):
        self.actual_position.append(msg.x)  # Extracting only X-coordinate
        self.time_data.append(time.time() - self.start_time)

def main():
    rclpy.init()
    node = TrackingPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

        plt.figure()
        plt.plot(node.time_data, node.setpoint_right, label="Right Motor Setpoint", linestyle="dashed")
        plt.plot(node.time_data, node.setpoint_left, label="Left Motor Setpoint", linestyle="dashed")
        plt.plot(node.time_data, node.actual_position, label="Actual Position (x)")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity / Position")
        plt.legend()
        plt.title("Tracking Performance: Setpoint vs Actual Position")
        plt.show()

if __name__ == '__main__':
    main()
