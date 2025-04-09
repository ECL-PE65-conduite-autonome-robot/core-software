import rclpy
from rclpy.node import Node
from core_interfaces.srv import ChooseCamera  # Assure-toi que l'import est correct

class CameraClient(Node):
    def __init__(self):
        super().__init__('camera_client')
        # Créer le client pour le service 'choose_camera'
        self.client = self.create_client(ChooseCamera, 'choose_camera')

        # Attendre que le service soit disponible
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service non disponible, attente...')

        # Créer une requête pour le service
        self.req = ChooseCamera.Request()

    def send_request(self, camera_name):
        # Remplir la requête avec le nom de la caméra
        self.req.camera_name = camera_name
        future = self.client.call_async(self.req)

        # Attendre la réponse du service
        rclpy.spin_until_future_complete(self, future)

        # Retourner la réponse
        return future.result()

def main(args=None):
    rclpy.init(args=args)

    client = CameraClient()
    camera_to_choose = "astra_camera"  # Nom de la caméra à tester

    # Envoyer la requête au service
    response = client.send_request(camera_to_choose)

    # Vérifier si la caméra est valide et afficher le résultat
    if response.camera_number != -1:
        client.get_logger().info(f"Numéro de la caméra '{camera_to_choose}': {response.camera_number}")
    else:
        client.get_logger().warn(f"Caméra '{camera_to_choose}' non reconnue.")

    # Fermer le client
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
