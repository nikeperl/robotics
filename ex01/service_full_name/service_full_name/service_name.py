#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fullname_msg_srv.srv import SummFullName

class FullNameService(Node):
    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(SummFullName, 'SummFullName', self.handle_summ)
        self.get_logger().info("Сервис 'SummFullName' запущен (node: service_name)")

    def handle_summ(self, request, response):
        full = f"{request.surname} {request.name} {request.patronymic}"
        response.full_name = full
        self.get_logger().info(f"Получено: '{request.surname}', '{request.name}', '{request.patronymic}' -> Отправляю: '{full}'")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FullNameService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
