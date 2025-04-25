import pytest
import rclpy

@pytest.fixture(scope='module')
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture()
def node(rclpy_init_shutdown):
    node = rclpy.create_node('test_node')
    yield node
    node.destroy_node()
