# Import required modules 
import rclpy
from rclpy.node import Node
from core_interfaces.srv import ChooseCamera, GetConfig  # Replace with your own .srv definitions if needed 
import json
from rclpy.executors import MultiThreadedExecutor

class CameraService(Node):
    def __init__(self):
        # Initialize the node with the name 'choose_camera'
        super().__init__('choose_camera')

        # Create the 'choose_camera' service with a callback function
        self.srv = self.create_service(ChooseCamera, 'choose_camera', self.callback)
        
        # Create a client for the 'get_config' service and wait until it is available 
        self.client = self.create_client(GetConfig, 'get_config')  # Nom du service
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Service not available, shutting down...')
                return
            self.get_logger().info('Waiting for service...')
        self.get_logger().info('Service available, ready to send requests!')


    def callback(self, request, response):
        """
        Service callback for 'choose_camera' service.
        Retrieves the camera name from the request, calls the 'get_config' service using a temporary node,
        logs the response and returns the appropriate camera number based on the mapping.
        """
        # Retrieve annd convert the requested camera name to lowercase 
        camera_name = request.camera_name.lower()
        self.get_logger().info(f"Request to choose camera: {camera_name}")

        # Create a temporary node to call the 'get_config' service 
        temp_node = rclpy.create_node('temp_client_node')
        # Call the temporary service function and get the result 
        result = call_get_config(temp_node)
        # Destroy the temporary node after the call is complete 
        temp_node.destroy_node()

        # Log the result from the 'get_config' service call if available 
        if result is not None:
            self.get_logger().info(f"Service : {result.config}")
        else:
            self.get_logger().warn("Failed to get response from GetParams service.")
        
        dic = json.loads(result.config)
        sensor_names = dic.keys()
        
        # Check if the camera name exits in the mapping and set the response accordingly
        if camera_name in sensor_names:
            self.get_logger().info(f"Camera '{camera_name}' -> Numéro {response.camera_number}")
        else:
            response.camera_number = -1 
            
            self.get_logger().warn(f"Camera '{camera_name}' non reconnue !")

        return response


def call_get_config(temp_node):
    client = temp_node.create_client(GetConfig, 'get_config')
    if not client.wait_for_service(timeout_sec=2.0):
        temp_node.get_logger().warn("Service get_config not available.")
        return None

    future = client.call_async(GetConfig.Request())
    rclpy.spin_until_future_complete(temp_node, future, timeout_sec=5)

    if future.done():
        return future.result()
    else:
        temp_node.get_logger().warn("Timeout in temp node")
        return None

 
def main(args=None):
    rclpy.init(args=args)
    node = CameraService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
