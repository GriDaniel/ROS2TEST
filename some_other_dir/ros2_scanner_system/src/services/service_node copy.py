from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalServices(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_intss', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        return response