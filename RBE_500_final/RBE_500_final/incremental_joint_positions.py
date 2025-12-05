import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from open_manipulator_msgs.srv import SetJointPosition
import numpy as np
import time

class IncrementalJointController(Node):
    def __init__(self):
        super().__init__("incremental_joint_positions_node")
        
        # Service client to command robot
        self.cli = self.create_client(
            SetJointPosition,
            '/goal_joint_space_path'
        )
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /goal_joint_space_path service...')
        
        # Subscribe to q_dot commands
        self.sub_qdot = self.create_subscription(
            Float64MultiArray,
            "/incremental_qdot_cmd",
            self.qdot_cmd_callback,
            10
        )
        
        # Initialize q1,q2,q3,q4 to 0
        self.q_ref = np.zeros(4)
        
        # Store current time for dt calculation 
        self.last_time = time.time()
        
        self.get_logger().info("Incremental joint controller ready (Service Mode)")
        
        # Keep updating every 0.1 seconds => 10Hz (service calls are slower)
        self.timer = self.create_timer(0.1, self.update_loop)
        
        # Initialize to zero
        self.q_dot_cmd = np.zeros(4)
        
        # Track if service is being called
        self.waiting_for_service = False
        self.service_future = None
    
    def qdot_cmd_callback(self, msg):
        """
        Receives joint velocity commands from the linear motion controller
        """
        self.q_dot_cmd = np.array(msg.data, dtype=float)
        self.get_logger().debug(f"Received q_dot: {self.q_dot_cmd}")
    
    def update_loop(self):
        # Skip if already waiting for service response
        if self.waiting_for_service:
            if self.service_future.done():
                try:
                    response = self.service_future.result()
                    if response and response.is_planned:
                        self.get_logger().debug("Command sent successfully")
                    else:
                        self.get_logger().warn("Command failed")
                except Exception as e:
                    self.get_logger().error(f"Service call error: {e}")
                finally:
                    self.waiting_for_service = False
            return
        
        # Calculate dt
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        # Incremental update
        self.q_ref = self.q_ref + self.q_dot_cmd * dt
        
        # Create service request
        req = SetJointPosition.Request()
        req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        req.joint_position.position = self.q_ref.tolist()
        req.path_time = 0.2  # Time to reach position (seconds)
        
        # Send async service call
        self.service_future = self.cli.call_async(req)
        self.waiting_for_service = True
        
        self.get_logger().debug(f"Commanded positions: {self.q_ref}")

def main(args=None):
    rclpy.init(args=args)
    node = IncrementalJointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()