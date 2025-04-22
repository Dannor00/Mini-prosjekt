import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult
from pid_controller_msgs.srv import SetReference
from std_msgs.msg import Float64, Float64MultiArray
import math

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # PID-parametre
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value

        self.get_logger().info(f"Bruker PID-parametre: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")

        self.dt = 0.01
        self.error_sum = 0.0
        self.prev_error = 0.0
        self.target_position = 0.0  # ønsket vinkel
        
        self.add_on_set_parameters_callback(self.parameter_callback)
        
       # Abonnerer på /reference for å sette ny ønsket vinkel
        self.srv = self.create_service(SetReference, 'set_reference', self.set_reference_callback)

        # Abonnerer på joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Publisher til velocity_controller
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10)

        self.get_logger().info("PID controller node har startet")
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp':
                if (param.value >= 0.0):
                    self.kp = param.value
                    self.get_logger().info(f' kp er satt til: {self.kp}')
            if param.name == 'ki':
                if (param.value >= 0.0):
                    self.ki = param.value
                    self.get_logger().info(f' ki er satt til: {self.ki}')
            if param.name == 'kd':
                if (param.value >= 0.0):
                    self.kd = param.value
                    self.get_logger().info(f' kd er satt til: {self.kd}')
                    
        return SetParametersResult(successful=True)
        
    def set_reference_callback(self, request, response):
        if -math.pi <= request.request <= math.pi:
            self.target_position = request.request
            self.get_logger().info(f"Ny referanseverdi satt til: {self.target_position:.2f} rad")
            response.success = True
        else:
            self.get_logger().warn(f"Ugyldig referanseverdi: {request.request:.2f}")
            response.success = False
            
        return response

    def joint_state_callback(self, msg):
        if not msg.position:
            self.get_logger().warn("Joint state has no position data")
            return

        position = msg.position[0]
        error = self.target_position - position
        self.error_sum += error * self.dt
        d_error = (error - self.prev_error) / self.dt

        control = self.kp * error + self.ki * self.error_sum + self.kd * d_error
        self.prev_error = error

        command_msg = Float64MultiArray()
        command_msg.data = [control]
        self.publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

