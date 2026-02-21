import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class ExtendedJacobianController(Node):
    def __init__(self):
        super().__init__('extended_jacobian_controller')

        # Tópicos solicitados
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_movil', 10)
        self.create_subscription(Odometry, '/odometry/global', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Estado: [x, y, psi]
        self.q = np.array([0.0, 0.0, 0.0])
        self.q_d = np.array([0.0, 0.0, 0.0]) # Deseado
        
        # Parámetros de la imagen
        self.a = 0.5  # Distancia al punto de control para evitar singularidad
        self.K1 = np.diag([0.4, 0.4, 0.6]) # Ganancias de saturación
        self.K2 = np.diag([1.2, 1.2, 1.0]) # Ganancias de convergencia
        self.vd = np.array([0.0, 0.0, 0.0]) # Velocidad deseada (feed-forward)

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.q = np.array([pos.x, pos.y, yaw])

    def goal_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.q_d = np.array([pos.x, pos.y, yaw])
        self.get_logger().info(f'Objetivo actualizado: {self.q_d}')

    def control_loop(self):
        psi = self.q[2]
        
        # 1. Definir el error considerando el punto desplazado 'a'
        # Esto es lo que permite que la Jacobiana sea cuadrada y no singular
        h = np.array([
            self.q[0],
            self.q[1],
            self.q[2]
        ])
        
        error = self.q_d - h
        
        # 2. Matriz Jacobiana Extendida J (de tu primera imagen)
        # Adaptada para el punto desplazado para que sea invertible
        # [x_dot, y_dot, psi_dot]^T = J * [u, v, w]^T
        J = np.array([
            [np.cos(psi), -np.sin(psi), -self.a * np.sin(psi)],
            [np.sin(psi),  np.cos(psi),  self.a * np.cos(psi)],
            [0.0,          0.0,          1.0]
        ])
        
        try:
            # 3. Cálculo de la Ley de Control (segunda imagen)
            # Vc = J^-1 * (Vd + K1 * tanh(K2 * error))
            J_inv = np.linalg.inv(J)
            control_term = self.K1 @ np.tanh(self.K2 @ error)
            vc = J_inv @ (self.vd + control_term)
            
            # vc contiene [u, v, omega]
            u_control = vc[0]
            v_control = vc[1] # Esta es la velocidad lateral teórica
            w_control = vc[2]
            
            # 4. Proyección para robot diferencial/skid-steering
            # Como el robot no tiene 'v' física, transformamos la cinemática
            # para que la omega final absorba la necesidad de corrección lateral
            v_lineal_final = u_control
            v_angular_final = w_control + (v_control / self.a)

            # 5. Publicación
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = float(v_lineal_final)
            msg.twist.angular.z = float(v_angular_final)
            
            self.pub_cmd.publish(msg)

            # Print de monitoreo para los estudiantes
            self.get_logger().info(
                f'\nError Global: {np.linalg.norm(error[:2]):.2f}m'
                f'\nVel calculadas [u, v, w]: [{vc[0]:.2f}, {vc[1]:.2f}, {vc[2]:.2f}]'
            )

        except np.linalg.LinAlgError:
            self.get_logger().error("Error: Matriz Jacobiana singular")

def main(args=None):
    rclpy.init(args=args)
    node = ExtendedJacobianController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()