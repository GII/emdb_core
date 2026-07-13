import random
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future

class ServiceClient(Node):
    """
    A generic service client class.
    """

    def __init__(self, service_type, service_name):
        """
        Constructor for the ServiceClient class.

        :param service_type: The type of the ROS 2 service.
        :type service_type: type
        :param service_name: The name of the ROS 2 service.
        :type service_name: str
        """        
        self.client_name = service_name.replace("/", "_") + '_client_' + f"{random.randint(0, 999999):06}"
        super().__init__(self.client_name)
        self.cbgroup=MutuallyExclusiveCallbackGroup()
        self.se = SingleThreadedExecutor()
        self.se.add_node(self)
        self.cli = self.create_client(service_type, service_name, callback_group=self.cbgroup)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {service_name} not available, waiting again...')
        self.req = service_type.Request()

    def send_request(self, **kwargs):
        """
        Send a request to the service.

        :param kwargs: Keyword arguments representing the request parameters.
        :type kwargs: dict
        :return: The response from the service.
        :rtype: service_type.Response
        """
        for key, value in kwargs.items():
            setattr(self.req, key, value)
        self.future = self.cli.call_async(self.req)
        self.se.spin_until_future_complete(self.future)
        return self.future.result()
    
class ServiceClientAsync():
    """
    A generic service client class.
    """
    def __init__(self, node:Node, service_type, service_name, callback_group) -> None:
        """
        Constructor for the ServiceClient class.

        :param node: ROS2 node that will host the service client.
        :type node: rclpy.node.Node
        :param service_type: Message Interface.
        :type service_type: Any ROS2 service interface
        :param service_name: Name of the service.
        :type service_name: str
        :param callback_group: Callback group to assign the service client.
        :type callback_group: rclpy.callback_groups.*
        """        
        self.node=node
        self.cli = self.node.create_client(service_type, service_name, callback_group=callback_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'Service {service_name} not available, waiting again...')
        self.req = service_type.Request()

    def send_request_async(self, **kwargs) -> Future:
        """
        Send a request to the service using asyncio.

        :param kwargs: Keyword arguments representing the request parameters.
        :type kwargs: dict
        :return: The response from the service.
        :rtype: service_type.Response
        """        
        for key, value in kwargs.items():
            setattr(self.req, key, value)
        self.future = self.cli.call_async(self.req)
        return self.future

def main():
    rclpy.init()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
