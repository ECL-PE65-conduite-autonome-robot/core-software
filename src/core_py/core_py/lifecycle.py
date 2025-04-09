from core_py.sensor import Sensor
from core_interfaces.srv import ReloadParams, StartSensor, StopSensor, GetParams, UpdateParam  # Import du service
import rclpy
from rclpy.node import Node
import yaml
from datetime import datetime
from pathlib import Path
import json 

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
        self.srv_update_param = self.create_service(UpdateParam, 'update_param', self.update_param_callback)
        self.sensors = []
        self.boot()

    def boot(self):
        self.get_logger().info(f"Booting up in mode: {self.__mode}...")

        self.load_parameters()
        self.launch_services()
        self.launch_sensors()
        self.get_logger().info(f"Boot completed in {datetime.now() - self.__start_date} seconds.")
        self.running()

    def running(self):
        self.get_logger().info(f"Running...")
        
    def launch_services(self):
        # Création des services
        self.get_logger().info(f"Creating services...")
        self.srv_reload = self.create_service(ReloadParams, 'reload_params', self.reload_parameters_callback)
        self.srv_start_sensor = self.create_service(StartSensor, 'start_sensor', self.start_sensor)
        self.srv_stop_sensor = self.create_service(StopSensor, 'stop_sensor', self.stop_sensor)
        self.srv_get_config = self.create_service(GetParams,'get_config',self.get_config)
        self.get_logger().info(f"Services launched !")
        
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
    
    def reload_parameters_callback(self, request, response):
        try:
            self.reload_dynamic_parameters()
            response.success = True
            response.message = "Parameters reloaded"
        except Exception as e:
            response.success = False
            response.message = f"Error reloading parameters: {e}"
        return response
    
    def launch_sensors(self):
        self.get_logger().info(f"Launching {len(self.params)} sensors...")
        
        for sensor_name, sensor_config in self.params.items():
            sensor = Sensor(sensor_config)
            self.sensors.append(sensor)
            if sensor.enabled:
                sensor.start_sensor()
            self.get_logger().info(f"Sensor {sensor_name} launched!")
        
        self.get_logger().info("(X/X) Sensors launched!")
    
    def start_sensor(self, request, response):
        self.get_logger().info(f"Requested to start sensor: {request.sensor_name} ...")
        for sensor in self.sensors:
            if sensor.name == request.sensor_name:
                response.success, response.message = sensor.start_sensor()
                return response
            
        response.success = False
        response.message = "Sensor not found"
        return response
    
    def stop_sensor(self, request, response):
        self.get_logger().info(f"Requested to stop sensor: {request.sensor_name} ...")
        for sensor in self.sensors:
            if sensor.name == request.sensor_name:
                response.success, response.message = sensor.stop_sensor()
                return response

        response.success = False
        response.message = "Sensor not found"
        return response
    
    def verify_config(self):
        pass

    def get_config(self,request,response):
        self.get_logger().info(f"Requested to send the current config.")
        response.params = json.dumps(self.params)
        return response
    
    def update_param_callback(self, request, response):
        sensor_name = request.sensor_name
        param_name = request.param_name
        new_value = request.new_value

        config_path = Path(self.__config_path, "dynamic_params.yml")
        if not config_path.exists():
            response.success = False
            response.message = "dynamic_params.yml not found"
            return response

        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            if sensor_name not in config:
                response.success = False
                response.message = "Sensor not found in config"
                return response

            if param_name not in config[sensor_name]["params"]:
                response.success = False
                response.message = "Parameter not found"
                return response

            # Convertir la valeur selon le type déclaré
            param_type = config[sensor_name]["params"][param_name]["type"]
            if param_type == "int":
                casted_value = int(new_value)
            elif param_type == "float":
                casted_value = float(new_value)
            elif param_type == "string":
                casted_value = str(new_value)
            else:
                response.success = False
                response.message = "Unsupported parameter type"
                return response

            config[sensor_name]["params"][param_name]["value"] = casted_value

            with open(config_path, "w") as f:
                yaml.dump(config, f)

            # Appel de la méthode du capteur actif
            for sensor in self.sensors:
                if sensor.name == sensor_name:
                    ok, msg = sensor.update_param(param_name, casted_value)
                    response.success = ok
                    response.message = msg
                    return response

            response.success = False
            response.message = "Sensor not instantiated"

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    

# Topic update config
# Service update param
# Action server change param, mode
#

def main(args=None):
    rclpy.init(args=args)
    life_cycle = LifeCycle()
    rclpy.spin(life_cycle)
    life_cycle.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
