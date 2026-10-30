import random
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from core.blocking_wait_check import check_blocking_wait

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
        self.service_name = service_name
        self.cli = self.create_client(service_type, service_name, callback_group=self.cbgroup)
        check_blocking_wait(f"ServiceClient waits for the service {service_name}", skip_files=(__file__,))
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
        check_blocking_wait(f"synchronous call to the service {self.service_name}", skip_files=(__file__,))
        for key, value in kwargs.items():
            setattr(self.req, key, value)
        self.future = self.cli.call_async(self.req)
        self.se.spin_until_future_complete(self.future)
        return self.future.result()
    
SERVICE_CHECK_PERIOD = 0.01  # Seconds between checks while an asynchronous client waits for its service
SERVICE_WAIT_WARNING_PERIOD = 5.0  # Seconds between warnings while an asynchronous client waits for its service


class ServiceClientAsync():
    """
    A generic service client class for callbacks (coroutines): nothing waits blocked.
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
        self.service_name = service_name
        self.cli = self.node.create_client(service_type, service_name, callback_group=callback_group)
        self.req = service_type.Request()
        # The timers of wait_for_service_async must run while the callback that waits is suspended, also with the
        # Python executors, which keep the callback group of a suspended callback busy
        self.wait_callback_group = ReentrantCallbackGroup()

    async def wait_for_service_async(self):
        """
        Waits until the client sees its service, with no time limit. It doesn't block the executor: between checks,
        the callback that waits is suspended and the executor runs other callbacks (e.g. the one that creates the node
        that provides the service). While waiting, it periodically logs a warning.
        """
        start = time.monotonic()
        next_warning = start + SERVICE_WAIT_WARNING_PERIOD
        while not self.cli.service_is_ready():
            await self._sleep(SERVICE_CHECK_PERIOD)
            if time.monotonic() >= next_warning:
                self.node.get_logger().warning(f"Waiting for the service {self.service_name} for {time.monotonic() - start:.1f} s")
                next_warning += SERVICE_WAIT_WARNING_PERIOD

    async def _sleep(self, seconds):
        """
        Suspends the calling coroutine for some seconds, without blocking the executor (one-shot timer).

        :param seconds: Time to wait.
        :type seconds: float
        """
        done = Future()

        def wake():
            timer.cancel()
            self.node.destroy_timer(timer)
            done.set_result(True)

        timer = self.node.create_timer(seconds, wake, callback_group=self.wait_callback_group)
        await done

    async def send_request_async(self, **kwargs):
        """
        Send a request to the service and wait for the response (coroutine, to be awaited). The first time, it waits
        until the client sees the service (see wait_for_service_async).

        :param kwargs: Keyword arguments representing the request parameters.
        :type kwargs: dict
        :return: The response from the service.
        :rtype: service_type.Response
        """        
        if not self.cli.service_is_ready():
            await self.wait_for_service_async()
        for key, value in kwargs.items():
            setattr(self.req, key, value)
        self.future = self.cli.call_async(self.req)
        return await self.future

def main():
    rclpy.init()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
