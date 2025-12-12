import rclpy
from rclpy.node import Node
import numpy as np
import time

from interfaces_pkg.srv import Joint4PositionRef  # your own service

from dynamixel_sdk_custom_interfaces.msg import SetCurrent
from dynamixel_sdk_custom_interfaces.srv import GetPosition


class Joint4PDController(Node):
    def __init__(self):
        super().__init__("joint4_pd_controller")

        # ----------------------------------------------------
        # USER-EDITABLE PARAMETERS (NO NEED FOR ROS2 CLI)
        # ----------------------------------------------------
        # PD GAINS 
        # These are in "current units per radian" (and per rad/s for Kd),
        # because the control output is directly Goal Current (102).
        self.Kp = 150.0
        self.Kd = 30.0

        # From Dynamixel docs: 1 unit ≈ 2.69 mA
        self.current_limit_units = 30   # SAFE CURRENT LIMIT (units)
        self.joint4_id = 14             # Joint 4 actuator ID
        self.log_path = "joint4_pd_log.txt"
        # ----------------------------------------------------

        # Conversion between ticks and radians for X-series
        self.TICKS_PER_REV = 4096.0
        self.CENTER_TICKS = 2048.0
        self.RAD_PER_TICK = 2.0 * np.pi / self.TICKS_PER_REV

        # State variables
        self.q4_meas = 0.0       # current joint4 position (rad)
        self.q4_dot_meas = 0.0   # current joint4 velocity (rad/s)
        self.q4_ref = 0.5     # desired position (rad)

        self.last_ticks = None
        self.last_time = None
        self.first_measurement_received = False

        self.t0 = time.time()

        # Stats for each reference step – to help tuning via terminal
        self.prev_ref = self.q4_ref
        self.step_start_time = self.t0
        self.max_abs_error = 0.0
        self.settled_reported = False
        self.settle_tol = 0.02      # [rad] ~1.1 degrees
        self.settle_time_window = 0.5  # need to stay in tol for this long
        self.last_in_tol_time = None

        # How often to print diagnostic line to terminal
        self.print_period = 0.1  # seconds
        self.last_print_time = self.t0

        # ----------------------------------------------------
        # Service client to read joint position (no /joint_states)
        # ----------------------------------------------------
        self.get_pos_client = self.create_client(
            GetPosition,
            "get_position"
        )

        # Publisher for Goal Current (102)
        # This topic is consumed by current_read_write_node
        self.current_pub = self.create_publisher(
            SetCurrent,
            "set_current",
            10,
        )

        # Service: set joint4 reference position (in radians)
        self.ref_srv = self.create_service(
            Joint4PositionRef,
            "set_joint4_reference",
            self.set_reference_cb,
        )

        # Log file for plotting in Matlab / report
        self.log_file = open(self.log_path, "w")
        self.log_file.write("# t, q_ref(rad), q_meas(rad), current_units, current_mA\n")

        # Control loop at ~100 Hz (0.01 s)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Joint4 PD controller node started.")
        self.get_logger().info(
            f"Kp={self.Kp}, Kd={self.Kd}, joint4_id={self.joint4_id}, "
            f"current_limit_units={self.current_limit_units}"
        )

    # Helper: convert Dynamixel position ticks to radians
    def ticks_to_rad(self, ticks: int) -> float:
        return (float(ticks) - self.CENTER_TICKS) * self.RAD_PER_TICK

    # SERVICE CALLBACK: set reference
    def set_reference_cb(self, request, response):
        """
        Service handler for setting new joint4 position reference (in radians).
        """
        self.q4_ref = float(request.q_ref)
        response.success = True
        response.message = f"New joint4 reference set to {self.q4_ref:.3f} rad"
        self.get_logger().info(response.message)

        # Reset step statistics for this new reference
        self.prev_ref = self.q4_ref
        self.step_start_time = time.time()
        self.max_abs_error = 0.0
        self.settled_reported = False
        self.last_in_tol_time = None

        return response

    # CALLBACK for GetPosition result
    def _get_position_done_cb(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"GetPosition call failed: {e}")
            return

        now = time.time()
        ticks = res.position
        q4_meas_new = self.ticks_to_rad(ticks)

        if self.last_time is None:
            # First measurement – do NOT overwrite q4_ref here.
            self.q4_meas = q4_meas_new
            self.q4_dot_meas = 0.0
            self.last_ticks = ticks
            self.last_time = now
            self.first_measurement_received = True

            self.get_logger().info(
                f"First GetPosition: q4_meas = {self.q4_meas:.3f} rad. "
                f"Current q4_ref = {self.q4_ref:.3f} rad."
            )
            return

        dt = now - self.last_time
        if dt > 0.0:
            q_prev = self.ticks_to_rad(self.last_ticks)
            self.q4_dot_meas = (q4_meas_new - q_prev) / dt

        self.q4_meas = q4_meas_new
        self.last_ticks = ticks
        self.last_time = now

    # CONTROL LOOP
    def control_loop(self):
        # 1) Request new position (async)
        if self.get_pos_client.service_is_ready():
            req = GetPosition.Request()
            req.id = int(self.joint4_id)
            future = self.get_pos_client.call_async(req)
            future.add_done_callback(self._get_position_done_cb)
        else:
            self.get_logger().warn(
                "get_position service not available yet...",
                throttle_duration_sec=2.0
            )
            return

        # Wait until we have at least one measurement
        if not self.first_measurement_received:
            return

        # 2) PD control
        e = self.q4_ref - self.q4_meas
        e_dot = -self.q4_dot_meas

        # Track max error for this reference step
        self.max_abs_error = max(self.max_abs_error, abs(e))

        raw_current_units = self.Kp * e + self.Kd * e_dot

        # Clamp to safe range
        current_units = max(
            -self.current_limit_units,
            min(self.current_limit_units, raw_current_units),
        )

        current_units_int = int(round(current_units))
        current_mA = current_units * 2.69

        # 3) Publish SetCurrent
        cmd = SetCurrent()
        cmd.id = int(self.joint4_id)
        cmd.current = current_units_int
        self.current_pub.publish(cmd)

        # 4) Detect “settled” region to help tuning
        now = time.time()
        if abs(e) < self.settle_tol:
            if self.last_in_tol_time is None:
                self.last_in_tol_time = now
            # If we've stayed in the tolerance band for long enough, report once
            elif (not self.settled_reported) and (now - self.last_in_tol_time >= self.settle_time_window):
                t_step = now - self.step_start_time
                self.get_logger().info(
                    f"[STEP RESULT] Settled at q_meas={self.q4_meas:.3f} rad "
                    f"for q_ref={self.q4_ref:.3f} rad in ~{t_step:.2f}s, "
                    f"max|error|≈{self.max_abs_error:.3f} rad"
                )
                self.settled_reported = True
        else:
            # Error went out of tolerance band again
            self.last_in_tol_time = None

        # 5) Print diagnostic line to terminal every print_period seconds
        if now - self.last_print_time >= self.print_period:
            t_since_start = now - self.t0
            self.get_logger().info(
                f"t={t_since_start:6.2f}s | q_ref={self.q4_ref:+.3f} rad | "
                f"q_meas={self.q4_meas:+.3f} rad | e={e:+.3f} | "
                f"qdot={self.q4_dot_meas:+.3f} rad/s | I={current_units_int:+4d}"
            )
            self.last_print_time = now

        # 6) Log for Matlab
        t_now = now - self.t0
        self.log_file.write(
            f"{t_now:.6f}, {self.q4_ref:.6f}, {self.q4_meas:.6f}, "
            f"{current_units:.6f}, {current_mA:.6f}\n"
        )

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
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
