import rclpy
from rclpy.node import Node
from core_interfaces.srv import ChooseCamera  # Remplace par ton service .srv

class CameraService(Node):
    def __init__(self):
        super().__init__('choose_camera')
        self.srv = self.create_service(ChooseCamera, 'choose_camera', self.callback)
        
        # Dictionnaire associant les caméras aux numéros
        self.camera_mapping = {
            "camera1": 1,
            "camera2": 2,
            "camera3": 3,
            "camera4": 4
        }

    def callback(self, request, response):
        camera_name = request.camera_name.lower()
        if camera_name in self.camera_mapping:
            response.camera_number = self.camera_mapping[camera_name]
            self.get_logger().info(f"Camera '{camera_name}' -> Numéro {response.camera_number}")
        else:
            response.camera_number = -1  # Code d'erreur si la caméra n'existe pas
            self.get_logger().warn(f"Camera '{camera_name}' non reconnue !")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CameraService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
