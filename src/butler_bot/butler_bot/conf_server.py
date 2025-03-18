import rclpy
from rclpy.node import Node
import time
from threading import Thread
from butler_interfaces.srv import TableConf
from butler_interfaces.srv import KitchenConf

class ConfServer(Node):

    def __init__(self):
        super().__init__('basic_server')

        self.kitchenconf = self.create_service(KitchenConf, 'kitchen_conf', self.handle_kitchen_request)
        self.tableconf = self.create_service(TableConf, 'table_conf', self.handle_table_request)
        self.get_logger().info('Service server is ready.')

    def input_with_timeout(self,prompt, timeout):
        result = None
        def input_function():
            nonlocal result
            result = input(prompt)
        
        input_thread = Thread(target=input_function)
        input_thread.daemon = True
        input_thread.start()
        input_thread.join(timeout)

        if input_thread.is_alive():
            print("Timeout: No input received in", timeout, "seconds.")
            return None
        else:
            return result

    def handle_kitchen_request(self, request, response):
        self.get_logger().info(f'kitchen Received request: order={request.order} enter ready orders')
        resp = self.input_with_timeout("Enter ready orders: ", 30)
        if resp:
            ready_orders = resp.split()
            response.ready = ready_orders
            self.get_logger().info(f'Received request: order={request.order}, ready={response.ready}')
        else:
            response.ready = []
        return response

    def handle_table_request(self, request, response):
        self.get_logger().info(f'Received request: table={request.tableid} enter conf')
        resp = self.input_with_timeout("Enter conf: ", 10)
        if resp:
            response.status = True
        else:
            response.status = False
        self.get_logger().info(f'Received request: table={request.tableid}, status={response.status}')
                
        return response


def main(args=None):
    rclpy.init(args=args)
    server = ConfServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Server shutting down.')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()