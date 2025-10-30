#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from fullname_msg_srv.srv import SummFullName

class FullNameClient(Node):
    def __init__(self):
        super().__init__('client_name')
        self.cli = self.create_client(SummFullName, 'SummFullName')

    def send_request(self, surname, name, patronymic):
        req = SummFullName.Request()
        req.surname = surname
        req.name = name
        req.patronymic = patronymic
        return self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    argv = sys.argv[1:]
    if len(argv) < 3:
        print("Usage: ros2 run service_full_name client_name <surname> <name> <patronymic>")
        return

    surname, name, patronymic = argv[0], argv[1], argv[2]
    node = FullNameClient()

    # ждём, пока сервис появится
    while not node.cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Сервис 'SummFullName' не доступен, ожидаю...")

    future = node.send_request(surname, name, patronymic)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Получен ответ: '{future.result().full_name}'")
        print(future.result().full_name)
    else:
        node.get_logger().error("Вызов сервиса завершился неудачей")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

