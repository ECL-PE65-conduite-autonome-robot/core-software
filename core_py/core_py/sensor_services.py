from core_py.core_py.constants import Constants
import rclpy
from rclpy.node import Node
from core_py.srv import ReloadParams  # Import du service
import yaml
from pathlib import Path

class SensorService(Node):
    def __init__(self):
        super().__init__('sensor_service')
        self.service = self.create_service(ReloadParams, 'reload_params', self.reload_params_callback)
        self.get_logger().info("Service 'reload_params' ready.")

    def reload_params_callback(self, request, response):
        config_path = Path(Constants.config_path, "dynamic_params.yml")

        if config_path.exists():
            try:
                with open(config_path, "r") as file:
                    new_params = yaml.safe_load(file)
                    self.get_logger().info(f"Loaded new parameters: {new_params}")
                    response.success = True
                    response.message = "Parameters reloaded successfully"
            except yaml.YAMLError as e:
                response.success = False
                response.message = f"Error parsing YAML: {e}"
        else:
            response.success = False
            response.message = "Config file not found"

        return response

def main():
    rclpy.init()
    node = SensorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
