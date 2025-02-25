import subprocess
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

    def running(self):
        self.get_logger().info(f"Running...")

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
        

class Sensor:
    def __init__(self, config):
        self.config = config
        self.name = config.get("name")
        self.description = config.get("description")
        self.type = config.get("type")
        self.package_name = config.get("package_name")
        self.node_name = config.get("node_name")
        self.launch_file = config.get("launch_file")
        self.params = config.get("params")
        self.process = None
        self.init()
    
    def init(self):
        print(f"Initializing sensor {self.name}...")
        print(f"Description: {self.description}")
        print(f"Type: {self.type}")
        print(f"Package: {self.package_name}")
        print(f"Node: {self.node_name}")
        print(f"Launch file: {self.launch_file}")
        print(f"Parameters: {len(self.params)}")
        print(f"Sensor {self.name} initialized!")
        
    def start_sensor(self):
        if not self.process:
            self.process = subprocess.Popen(["ros2", "launch", self.package_name, self.launch_file])
            response = (True, "Sensor started")
        else:
            response = (False, "Sensor already running")
        return response

    def stop_sensor(self):
        if self.process:
            self.process.terminate()
            self.process.wait()
            self.process = None
            response = (True, "Sensor stopped")
        else:
            response = (False, "Sensor not running")
        return response

def main(args=None):
    rclpy.init(args=args)
    life_cycle = LifeCycle()
    rclpy.spin(life_cycle)
    life_cycle.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
