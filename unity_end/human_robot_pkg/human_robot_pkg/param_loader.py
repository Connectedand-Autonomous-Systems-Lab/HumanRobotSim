import rclpy
from rclpy.node import Node
import yaml

class ParameterLoaderNode(Node):
    def __init__(self):
        super().__init__('parameter_loader_node')

        # Load parameters from YAML file
        yaml_file = 'src/DRL-exploration/unity_end/human_robot_pkg/param/params.yaml'
        self.load_parameters_from_yaml(yaml_file)

    def load_parameters_from_yaml(self, yaml_file):
        try:
            with open(yaml_file, 'r') as file:
                params = yaml.safe_load(file)
                self.declare_nested_parameters('', params)
                self.get_logger().info('YAML parameters loaded successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to load YAML file: {e}')

    def declare_nested_parameters(self, namespace, params):
        """Recursively declare nested parameters."""
        for key, value in params.items():
            full_key = f"{namespace}.{key}" if namespace else key
            if isinstance(value, dict):
                self.declare_nested_parameters(full_key, value)
            else:
                self.declare_parameter(full_key, value)
                self.get_logger().info(f'Loaded parameter: {full_key} = {value}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterLoaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()