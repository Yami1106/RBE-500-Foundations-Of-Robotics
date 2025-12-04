import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from interfaces_pkg.srv import EEVeltoJointVel


class LinearMotionController(Node):
    def __init__(self):
        super().__init__("linear_motion_controller")
        
        # Current joint state
        self.q_current = None
        
        # State management for non-blocking service calls
        self.waiting_for_service = False
        self.service_future = None
        
        # Subscribe to joint states
        self.sub_js = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10
        )
        
        # Publisher to incremental controller
        self.pub_qdot = self.create_publisher(
            Float64MultiArray,
            "/incremental_qdot_cmd",
            10
        )
        
        # Service client
        self.cli = self.create_client(
            EEVeltoJointVel,
            "ee_vel_to_joint_vel"
        )
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for ee_vel_to_joint_vel service...")
        
        # Constant +y velocity (m/s)
        self.x_dot_cmd = np.array([0.0, 0.03, 0.0,   # linear (vx, vy, vz)
                                   0.0, 0.0, 0.0])  # angular (wx, wy, wz)
        
        # Timer loop - 50 Hz
        self.timer = self.create_timer(0.02, self.timer_cb)
        
        self.get_logger().info("Linear motion controller ready.")
        self.get_logger().info("Commanding velocity: +Y = 0.03 m/s")
    
    def joint_state_cb(self, msg: JointState):
        """Update current joint positions"""
        if len(msg.position) >= 4:
            self.q_current = np.array(msg.position[:4], dtype=float)
    
    def timer_cb(self):
        """State machine: handles service calls without blocking"""
        
        # STATE 1: Waiting for joint states
        if self.q_current is None:
            self.get_logger().warn("Waiting for joint states...", throttle_duration_sec=2.0)
            return
        
        # STATE 2: Already waiting for a service response
        if self.waiting_for_service:
            # Check if service response is ready
            if self.service_future.done():
                self.handle_service_response()
                self.waiting_for_service = False
            else:
                # Still waiting, skip this cycle
                return
        
        # STATE 3: Ready to send new service request
        else:
            self.send_service_request()
    
    def send_service_request(self):
        """Send service request (non-blocking)"""
        req = EEVeltoJointVel.Request()
        req.q = self.q_current.tolist()
        req.x_dot = self.x_dot_cmd.tolist()
        
        # Send async request
        self.service_future = self.cli.call_async(req)
        self.waiting_for_service = True
    
    def handle_service_response(self):
        """Process service response and publish"""
        try:
            response = self.service_future.result()
            
            if response is None:
                self.get_logger().warn("No response from service")
                return
            
            # Get joint velocities
            q_dot = np.array(response.q_dot, dtype=float)
            
            # Publish to incremental controller
            msg = Float64MultiArray()
            msg.data = q_dot.tolist()
            self.pub_qdot.publish(msg)
            
            self.get_logger().debug(f"Published q_dot: {q_dot}")
            
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LinearMotionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()