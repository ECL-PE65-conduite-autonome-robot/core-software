import rclpy
import threading
import time
from rclpy.node import Node
from core_interfaces.srv import ChooseCamera, SetMode, SetStatus, ReloadParams
import pytest
# Importer votre lifecycle node
from core_py.lifecycle import LifeCycle  # Adjusted import to correct module path
from std_msgs.msg import String # For publishing parameter updates in JSON format

@pytest.fixture(scope='module')
def rclpy_node():
    rclpy.init()
    # Démarrer le Lifecycle Node en arrière-plan
    lifecycle_node = LifeCycle()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(lifecycle_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(2)  # Augmentation du délai pour laisser le temps au service d'être annoncé
    
    # Création du node de test
    node = rclpy.create_node('test_lifecycle')
    yield node
    node.destroy_node()
    lifecycle_node.destroy_node()
    rclpy.shutdown()
    
    
def test_change_mode(rclpy_node):
    client = rclpy_node.create_client(SetMode, "/set_mode")
    
    # Wait for service to be available
    assert client.wait_for_service(timeout_sec=5.0), "Service not available"

    # Create request
    request = SetMode.Request()
    request.mode = 123

    # Send request and wait for result
    future = client.call_async(request)
    rclpy.spin_until_future_complete(rclpy_node, future)

    assert future.result() is not None, "Service call failed"
    response = future.result()
    assert not response.success, f"Expected error, not a good mode"
    
    for i in range(3):
        request.mode = i
        future = client.call_async(request)
        rclpy.spin_until_future_complete(rclpy_node, future)
        assert future.result() is not None, f"Service call failed, mode {i}"
        response = future.result()
        assert response.success, f"Expected success, got {response.message} for mode {i}"

    client.destroy()

def test_change_status(rclpy_node):
    client = rclpy_node.create_client(SetStatus, "/set_status")
    
    # Wait for service to be available
    assert client.wait_for_service(timeout_sec=5.0), "Service not available"

    # Create request
    request = SetStatus.Request()
    request.status = 123

    # Send request and wait for result
    future = client.call_async(request)
    rclpy.spin_until_future_complete(rclpy_node, future)

    assert future.result() is not None, "Service call failed"
    response = future.result()
    assert not response.success, f"Expected error, not a good status"
    
    for i in range(3):
        request.status = i
        future = client.call_async(request)
        rclpy.spin_until_future_complete(rclpy_node, future)
        assert future.result() is not None, f"Service call failed, status {i}"
        response = future.result()
        assert response.success, f"Expected success, got {response.message} for status {i}"

    client.destroy()

def test_reload_parameters(rclpy_node):
    client = rclpy_node.create_client(ReloadParams, "/reload_params")
    
    # Wait for service to be available
    assert client.wait_for_service(timeout_sec=5.0), "Service not available"

    # Create request
    request = ReloadParams.Request()

    # Subscribing to topic /params_update to receive parameter updates
    received_msgs = []
    subscription = rclpy_node.create_subscription(
         String, '/params_update',
         lambda msg: received_msgs.append(msg),
         10)
    
    # Send request and wait for result
    future = client.call_async(request)
    rclpy.spin_until_future_complete(rclpy_node, future, timeout_sec=5.0)

    assert future.result() is not None, "Service call failed"
    response = future.result()
    assert response.success, f"Expected error, not a good status"
    
    # Wait for the parameter update message to be received
    # This is a blocking wait, adjust the timeout as needed
    timeout = time.time() + 5.0 # 5 seconds timeout
    while not received_msgs and time.time() < timeout:
        time.sleep(0.1)
    assert len(received_msgs) != 0, "Did not receive update params message on /params_update"
