import subprocess
from core_py.sensor import Sensor
from core_py.srv import ReloadParams, StartSensor, StopSensor  # Import du service
import rclpy
from rclpy.node import Node
import yaml
from datetime import datetime
from pathlib import Path

class LifeCycle(Node):
    def __init__(self):
        super().__init__("life_cycle")
        self.__mode = 0  # 0 Manual - 1 Autonome - 2 Debug
        self.__start_date = datetime.now()
        self.__status = 0  # 0 Off - 1 On - 2 Pause
        self.__exit_requested = False  # To kill the node completely
        self.params = {}  # Dictionnaire pour stocker les paramètres
        self.__config_path = Path("config", "parameters")
        self.dynamic_config = {}
        self.static_config = {}
        self.sensors = []
        self.boot()

    def boot(self):
        self.get_logger().info(f"Booting up in mode: {self.__mode}...")

        self.load_parameters()
        self.get_logger().info(f"Boot completed in {datetime.now() - self.__start_date} seconds.")
        self.launch_sensors()
        self.running()
        self.launch_services()

    def running(self):
        self.get_logger().info(f"Running...")
        
    def launch_services(self):
        # Création des services
        self.get_logger(f"Creating services...")
        self.srv_reload = self.create_service(ReloadParams, 'reload_params', self.reload_parameters_callback)
        self.srv_start_sensor = self.create_service(StartSensor, 'start_sensor', self.start_sensor_callback)
        self.srv_stop_sensor = self.create_service(StopSensor, 'stop_sensor', self.stop_sensor_callback)
        self.get_logger(f"Services launched !")
        
    def load_parameters(self):
        self.get_logger().info(f"Loading parameters...")

        # Définir les chemins des fichiers de configuration
        config_files = ["static_params.yml", "dynamic_params.yml"]
        
        for config_file in config_files:
            config_path = Path(self.__config_path, config_file)
            print(config_path.absolute())
            if config_path.exists():
                self.get_logger().info(f"Loading config: {config_path}")
                
                with open(config_path, "r") as file:
                    try:
                        config_data = yaml.safe_load(file)
                        self.params.update(config_data)  # Stocke les paramètres
                    except yaml.YAMLError as e:
                        self.get_logger().error(f"Error loading {config_path}: {e}")
            else:
                self.get_logger().warn(f"Config file {config_file} not found!")

        # Afficher les paramètres chargés
        self.get_logger().info(f"Loaded parameters: {self.params}")

    def reload_dynamic_parameters(self):
        self.get_logger().info(f"Reloading dynamic parameters...")
        dynamic_config_path = Path(self.__config_path,"dynamic_params.yml")

        if dynamic_config_path.exists():
            with open(dynamic_config_path, "r") as file:
                try:
                    dynamic_params = yaml.safe_load(file)
                    self.params.update(dynamic_params)  # Met à jour uniquement les paramètres dynamiques
                    self.get_logger().info(f"Updated dynamic parameters: {dynamic_params}")
                except yaml.YAMLError as e:
                    self.get_logger().error(f"Error loading dynamic_params.yml: {e}")
        else:
            self.get_logger().warn("Dynamic config file not found!")
            
    def launch_sensors(self):
        self.get_logger().info(f"Launching {len(self.params)} sensors...")
        
        for sensor_name, sensor_config in self.params.items():
            sensor = Sensor(sensor_config)
            self.sensors.append(sensor)
            sensor.start_sensor()
            self.get_logger().info(f"Sensor {sensor_name} launched!")
        
        self.get_logger().info("(X/X) Sensors launched!")
    
    def verify_config(self):
        pass
    

# Topic update config
# Service reload config
# Service update param
# Action server change param, mode
#

a=0

def main(args=None):
    rclpy.init(args=args)
    life_cycle = LifeCycle()
    rclpy.spin(life_cycle)
    life_cycle.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
