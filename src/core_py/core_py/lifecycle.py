# Import required modules and services 
from ament_index_python import get_package_share_directory
from core_py.sensor import Sensor
from core_interfaces.srv import Reboot, ReloadParams, StartSensor, StopSensor, GetConfig, SetStatus, SetMode, UpdateParam  # ROS 2 services 
import rclpy
from rclpy.node import Node
import yaml
from datetime import datetime
from pathlib import Path
import json 
from std_msgs.msg import String # For publishing parameter updates in JSON format

class LifeCycle(Node):
    def __init__(self):
        # Initialize the node with the name "life_cycle"
        super().__init__("life_cycle")

        # Define operating variables 
        self.__mode = 0  # Operating mode : 0 = Manual, 1 = Autonomous, 2 = Debug
        self.__start_date = datetime.now() # Timestamp for node startup 
        self.__status = 0  # Node status : 0 = Off, 1 = On, 2 = Pause
        self.__exit_requested = False  # Flag to indicate complete node shutdown requested
        self.params = {}  # Dictionary for storing parameters 
        self.__config_path = Path(get_package_share_directory("core_py"), "config", "parameters") # Path to the configuration files 
        self.dynamic_config = {} # Dictionary for dynamic parameters 
        self.static_config = {} # Dictionary for static parameters
        self.srv_update_param = self.create_service(UpdateParam, 'update_param', self.update_param_callback)
        self.sensors = [] # list to store sensor instances 

        # Start the lifecycle process
        self.boot()

    def boot(self):
        # Log the startup process and current operating mode
        self.get_logger().info(f"Booting up in mode: {self.__mode}...")
        
        # Launch topics, load parameters, launch services, and start sensors
        self.launch_topics() # Create publishers / subscribers 
        self.load_parameters() # Load parameters from YAML configuration file
        self.launch_services() # Set up ROS2 services
        self.launch_sensors() # Initialize sensors based on parameters 
        self.get_logger().info(f"Boot completed in {datetime.now() - self.__start_date} seconds.")
        self.running() # Transition to running state 
    
    def reboot(self, request, response):
        """
        Service callback to reboot the lifecycle node.
        This method performs a soft reboot of the lifecycle by :
        - Stopping all sensors
        - Resetting the sensor list
        - Reploading parameters
        - Relaunching sensors
        """
        self.get_logger().info("Reboot requested. Initiating reboot sequence...")

        # Stop all sensors
        for sensor in self.sensors:
            try:
                sensor.stop_sensor()
                self.get_logger().info(f"Sensor {sensor.name} stopped")
            except Exception as e: 
                self.get_logger().error(f"Error stopping sensor{sensor.name}: {e}")
        
        # Reset the sensor list
        self.sensors = []

        # Reload all parameters (static and dynamic)
        self.load_parameters()

        # Relaunch sensors according to the configuration 
        self.launch_sensors()

        response.success = True
        response.message = "Lifecycle rebooted successfully"
        return response
        

    def running(self):
        # Log that the node is now running
        self.get_logger().info(f"Running...")
        
    def launch_topics(self):
        # Create a publisher for sending parameter updates 
        self.params_pub = self.create_publisher(String, 'params_update', 10)

        
    def launch_services(self):
        # Create and launch the ROS2 services 
        self.get_logger().info(f"Creating services...")
        self.srv_reload = self.create_service(ReloadParams, 'reload_params', self.reload_parameters_callback)
        self.srv_start_sensor = self.create_service(StartSensor, 'start_sensor', self.start_sensor)
        self.srv_stop_sensor = self.create_service(StopSensor, 'stop_sensor', self.stop_sensor)
        self.srv_get_config = self.create_service(GetConfig,'get_config',self.get_config)
        self.srv_set_mode = self.create_service(SetMode,'set_mode',self.set_mode)
        self.srv_set_status = self.create_service(SetStatus,'set_status',self.set_status)
        self.srv_reboot = self.create_service(Reboot,'reboot',self.reboot)
        self.get_logger().info(f"Services launched !")
        
    def load_parameters(self):
        # Load parameters from YAML configuration files 
        self.get_logger().info(f"Loading parameters...")
        self.get_logger().info(f"Config path: {self.__config_path.absolute()}")

        # Define the list of configuration files to load 
        config_files = ["static_params.yml", "dynamic_params.yml"]
        
        # Iterate over each configuration file
        for config_file in config_files:
            config_path = Path(self.__config_path, config_file)
            if config_path.exists():
                self.get_logger().info(f"Loading config: {config_path}")
                
                with open(config_path, "r") as file:
                    try:
                        # Load data from the YAML file and update the parameters dictionary 
                        config_data = yaml.safe_load(file)
                        self.params  = merge_dicts(self.params, config_data)  # Stocke les paramètres
                    except yaml.YAMLError as e:
                        self.get_logger().error(f"Error loading {config_path}: {e}")
            else:
                self.get_logger().warn(f"Config file {config_file} not found!")
            
        # Log the loaded parameters 
        self.get_logger().info(f"Loaded parameters: {self.params}")
        # Call to publish the parameters once loaded 
        self.publish_params()

    def reload_dynamic_parameters(self):
        # Reload dynamic parameters from the corresponding YAML file 
        self.get_logger().info(f"Reloading dynamic parameters...")
        dynamic_config_path = Path(self.__config_path,"dynamic_params.yml")

        if dynamic_config_path.exists():
            with open(dynamic_config_path, "r") as file:
                try:
                    # Load dynamic parameters and update the main parameters dictionary 
                    dynamic_params = yaml.safe_load(file)
                    self.params.update(dynamic_params)  # Met à jour uniquement les paramètres dynamiques
                    self.get_logger().info(f"Updated dynamic parameters: {dynamic_params}")
                    # Call to publish the update after loading the parameters 
                    self.publish_params()
                except yaml.YAMLError as e:
                    self.get_logger().error(f"Error loading dynamic_params.yml: {e}")
        else:
            self.get_logger().warn("Dynamic config file not found!")
    
    def reload_parameters_callback(self, request, response):
        # Service callback to reload parameters 
        try:
            self.reload_dynamic_parameters()
            response.success = True
            response.message = "Parameters reloaded"
        except Exception as e:
            response.success = False
            response.message = f"Error reloading parameters: {e}"
        return response
    
    def launch_sensors(self):
        # Initialize sensors based on the loaded parameters 
        self.get_logger().info(f"Launching {len(self.params)} sensors...")
        
        for sensor_name, sensor_config in self.params.items():
            sensor = Sensor(sensor_config)
            self.sensors.append(sensor)
            if sensor.enabled:
                sensor.start_sensor() # Start the sensor if enabled 
                self.get_logger().info(f"Sensor {sensor_name} launched!")
            else:
                self.get_logger().info(f"Sensor {sensor_name} is disabled in config.")
        
        self.get_logger().info("(X/X) Sensors launched!")
    
    def start_sensor(self, request, response):
        # Service to start a specific sensor by name 
        self.get_logger().info(f"Requested to start sensor: {request.sensor_name} ...")
        for sensor in self.sensors:
            if sensor.name == request.sensor_name:
                response.success, response.message = sensor.start_sensor()
                return response
            
        response.success = False
        response.message = "Sensor not found"
        return response
    
    def stop_sensor(self, request, response):
        # Service to stop a specific sensor by name 
        self.get_logger().info(f"Requested to stop sensor: {request.sensor_name} ...")
        for sensor in self.sensors:
            if sensor.name == request.sensor_name:
                response.success, response.message = sensor.stop_sensor()
                return response

        response.success = False
        response.message = "Sensor not found"
        return response
    
    def verify_config(self):
        # Method intended to verify the configuration coherence (implementation pending)
        pass

    def get_config(self,request,response):
        # Service to send the current configuration in JSON format
        self.get_logger().info(f"Requested to send the current config.")
        response.config = json.dumps(self.params)
        return response
    
    def set_status(self,request, response):
        # Service to change the node's status (Off, On, Pause)
        if request.status in [0,1,2]:
            self.__status = request.status
            self.get_logger().info(f"Changed status to : {self.__status}")
            response.success = True
            response.message = "Status change"
        else:
            response.success = False
            response.message = "Request not valid"

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


    def set_mode(self,request, response):
        # Service to change the operating mode (Manual, Autonomous, Debug)
        if request.mode in [0,1,2]:
            self.__mode = request.mode
            self.get_logger().info(f"Changed mode to : {self.__mode}")
            response.success = True
            response.message = "Mode changed"
        else:
            response.success = False
            response.message = "Request not valid"

        return response           
 

    # Method that publish the updated parameters dictionary (in JSON format) on the 'param_update' topic
    def publish_params(self):
        # Log the parameter update publication
        self.get_logger().info("Publishing parameters update")
        # Create a String message and convert the parameters dictionary to JSON 
        msg = String()
        msg.data = json.dumps(self.params)
        # Publish the message on the 'params_update' topic 
        self.params_pub.publish(msg)
        

def merge_dicts(d1, d2):
    result = dict(d1)  # Make a shallow copy of the first dict
    for key, value in d2.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = merge_dicts(result[key], value)
        else:
            result[key] = value
    return result



# Main entry point for the script 
def main(args=None):
    # Initalize rclpy 
    rclpy.init(args=args)
    # Instantiate the LifeCycle node
    life_cycle = LifeCycle()
    # Enter the ROS2 spinning loop
    rclpy.spin(life_cycle)
    # Destroy the node and shut down rclpy when finished 
    life_cycle.destroy_node()
    rclpy.shutdown()


# Execute main() if this is run directly 
if __name__ == "__main__":
    main()
