#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class TextToCmdVel(Node):
    def __init__(self):
        super().__init__('text_to_cmd_vel')
        # Подписываемся на текстовые команды
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.listener_callback,
            10)
        self.subscription  # предотвращает удаление переменной сборщиком мусора

        # Публикатор в /turtle1/cmd_vel
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.get_logger().info("Узел text_to_cmd_vel запущен и слушает топик 'cmd_text'.")

    def listener_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        twist = Twist()

        if cmd == "move_forward":
            twist.linear.x = 1.0
            self.get_logger().info("Движение вперёд")
        elif cmd == "move_backward":
            twist.linear.x = -1.0
            self.get_logger().info("Движение назад")
        elif cmd == "turn_left":
            twist.angular.z = 1.5
            self.get_logger().info("Поворот влево")
        elif cmd == "turn_right":
            twist.angular.z = -1.5
            self.get_logger().info("Поворот вправо")
        else:
            self.get_logger().warn(f"Неизвестная команда: {cmd}")
            return

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TextToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

