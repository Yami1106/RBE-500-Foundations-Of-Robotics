import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from interfaces_pkg.srv import EEVeltoJointVel

class LinearMotionController(Node):
    def __init__(self):
        super().__init__("linear_motion_controller")
        
        # Current joint state (from /joint_states)
        self.q_current = None  
        
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
        
        # EE -> joint velocities
        self.cli = self.create_client(
            EEVeltoJointVel,
            "ee_vel_to_joint_vel"
        )
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for ee_vel_to_joint_vel service...")
        
        # Constant +y velocity (m/s)
        self.x_dot_cmd = np.array([0.0, 0.03, 0.0,   # linear (vx, vy, vz)
                                   0.0, 0.0, 0.0])  # angular (wx,wy,wz)
        
        # Timer loop 
        self.timer = self.create_timer(0.02, self.timer_cb)
        
        self.get_logger().info("Linear motion controller ready.")
        self.get_logger().info("Commanding velocity: +Y = 0.03 m/s")
    
    def joint_state_cb(self, msg: JointState):
        if len(msg.position) >= 4:
            self.q_current = np.array(msg.position[:4], dtype=float)
    
    def timer_cb(self):
        # Check if we've received joint states
        if self.q_current is None:
            self.get_logger().warn("Waiting for joint states...", throttle_duration_sec=2.0)
            return
        
        # Prepare service request
        req = EEVeltoJointVel.Request()
        req.q = self.q_current.tolist()
        req.x_dot = self.x_dot_cmd.tolist()
        
        # Call service
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is None:
            self.get_logger().warn("No response from ee_vel_to_joint_vel")
            return
        
        # Get joint velocities from service response
        q_dot = np.array(future.result().q_dot, dtype=float)
        
        # Publish to incremental controller
        msg = Float64MultiArray()
        msg.data = q_dot.tolist()
        self.pub_qdot.publish(msg)
        
        self.get_logger().debug(f"Published q_dot: {q_dot}")

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