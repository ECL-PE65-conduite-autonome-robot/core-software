import rclpy
from rclpy.node import Node
from core_interfaces.srv import ChooseCamera, GetParams  # Remplace par ton service .srv
import json

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
        
        self.client = self.create_client(GetParams, 'get_config')  # Nom du service
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Service not available, shutting down...')
                return
            self.get_logger().info('Waiting for service...')
        self.get_logger().info('Service available, ready to send requests!')
        self.send_request("camera1")  # Exemple d'appel de service avec une caméra spécifique

    def send_request(self, camera_name):
        request = GetParams.Request()
        self.get_logger().info(f"Envoi de la requête pour la caméra : {camera_name}")
        
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Réponse du service : {response.params}")  # Modifier selon la structure de `GetParams.Response`
            json_response = json.loads(response.params)
            
            for camera in json_response.keys():
                self.get_logger().info(f"Camera: {camera}")
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'appel du service : {e}")
            
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
