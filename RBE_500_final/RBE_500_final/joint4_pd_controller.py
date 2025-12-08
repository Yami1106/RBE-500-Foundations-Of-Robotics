import rclpy
from rclpy.node import Node
import numpy as np
import time

from sensor_msgs.msg import JointState
from interfaces_pkg.srv import Joint4PositionRef

# from the Dynamixel SDK custom interfaces package
from dynamixel_sdk_custom_interfaces.msg import SetCurrent


class Joint4PDController(Node):
    def __init__(self):
        super().__init__("joint4_pd_controller")
        # tune the values 
        self.Kp = 1.0  
        self.Kd = 0.0   

        self.q4_meas = 0.0        # current joint4 position (rad)
        self.q4_dot_meas = 0.0    # current joint4 velocity (rad/s)
        self.q4_ref = 0.0         # desired position (set via service)

        self.last_time = time.time()
        self.last_q4 = 0.0

        self.joint4_id = 4

        # Subscribe to /joint_states
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10
        )
        # set_current is from the dynamixel example package
        self.current_pub = self.create_publisher(
            SetCurrent,
            "set_current",
            10
        )

        self.ref_srv = self.create_service(
            Joint4PositionRef,
            "set_joint4_reference",
            self.set_reference_cb
        )

        self.log_file = open("joint4_pd_log.txt", "w")
        self.log_file.write("# t, q_ref, q_meas, effort(current_units)\n")

        # e.g. 100 Hz (0.01 s)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Joint4 PD controller node started.")
        self.get_logger().info("Publishing current commands on 'set_current'.")

    # /joint_states callback
    def joint_state_cb(self, msg: JointState):
        """
        Get joint 4 position and velocity from /joint_states.

        Assumes:
        - 'joint4' exists in msg.name
        - msg.position and msg.velocity aligned with msg.name
        """
        try:
            idx = msg.name.index("joint4")
        except ValueError:
            # 'joint4' not seen yet in this message
            return

        self.q4_meas = msg.position[idx]

        if len(msg.velocity) > idx and not np.isnan(msg.velocity[idx]):
            # Use velocity field if available
            self.q4_dot_meas = msg.velocity[idx]
        else:
            current_time = time.time()
            dt = current_time - self.last_time
            if dt > 0.0:
                self.q4_dot_meas = (self.q4_meas - self.last_q4) / dt
            self.last_time = current_time
            self.last_q4 = self.q4_meas

    def set_reference_cb(self, request, response):
        """
        Service handler for setting new joint4 position reference.
        """
        self.q4_ref = float(request.q_ref)
        response.success = True
        response.message = f"New joint4 reference set to {self.q4_ref:.3f} rad"
        self.get_logger().info(response.message)
        return response
    
    def control_loop(self):
        """
        Runs at fixed frequency (dt ~ 0.01 s).
        Computes PD control and publishes current to joint 4.
        """
        # Position error
        e = self.q4_ref - self.q4_meas

        # Desired final velocity is 0 at goal, so velocity error = -q_dot
        e_dot = -self.q4_dot_meas

        # PD control : tau ~ desired current (in Dynamixel current units)
        tau = self.Kp * e + self.Kd * e_dot

        # need to adjust values
        max_current = 1  # check model specs before using
        tau = float(max(min(tau, max_current), -max_current))

        # Build and publish SetCurrent message
        cmd = SetCurrent()
        cmd.id = int(self.joint4_id)
        cmd.current = int(tau)  

        self.current_pub.publish(cmd)

        # Log for plotting in matlab
        t_now = time.time()
        self.log_file.write(
            f"{t_now:.6f}, {self.q4_ref:.6f}, {self.q4_meas:.6f}, {tau:.6f}\n"
        )
        # self.log_file.flush()

    def destroy_node(self):
        if not self.log_file.closed:
            self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Joint4PDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
