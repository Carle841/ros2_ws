import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

class ServicePractice(Node):
    def __init__(self):
        super().__init__('service_practice')
        self.pen_color = (0, 255, 0)  # Inicialmente verde
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio...')
        self.change_pen_color(*self.pen_color)

    def pose_callback(self, msg):
        if msg.x > 5.5 and self.pen_color != (255, 0, 0):
            self.change_pen_color(255, 0, 0)  # Cambiar a rojo
        elif msg.x <= 5.5 and self.pen_color != (0, 255, 0):
            self.change_pen_color(0, 255, 0)  # Cambiar a verde

    def change_pen_color(self, r, g, b):
        self.get_logger().info(f'Cambiando el color de pluma a: ({r}, {g}, {b})')
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = 2

        future = self.client.call_async(request)
        future.add_done_callback(lambda f: self.handle_service_response(f, r, g, b))

    def handle_service_response(self, future, r, g, b):
        if future.result() is not None:
            self.pen_color = (r, g, b)
            self.get_logger().info(f'Color de pluma cambiado a: ({r}, {g}, {b})')
        else:
            self.get_logger().error('Error al llamar al servicio')

def main(args=None):
    rclpy.init(args=args)
    node = ServicePractice()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
