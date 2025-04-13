import subprocess

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
        self.enabled = config.get("enabled")
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
        # TODO: Modify the static configuration file (enabled = True)
        if not self.process:
            try:
                self.process = subprocess.Popen(["ros2", "launch", self.package_name, self.launch_file])
                response = (True, "Sensor started")
            except Exception as e:
                response = (False, f"Error starting sensor: {e}")
        else:
            response = (False, "Sensor already running")
        return response

    def stop_sensor(self):
        # TODO: Modify the static configuration file (enabled = False)

        if self.process:
            self.process.terminate()
            self.process.wait()
            self.process = None
            response = (True, "Sensor stopped")
        else:
            response = (False, "Sensor not running")
        return response

    def update_param(self, param_name, new_value):
        if param_name in self.config["params"]:
            self.config["params"][param_name]["value"] = new_value
            return True, f"Parameter {param_name} updated to {new_value}"
        return False, f"Parameter {param_name} not found"