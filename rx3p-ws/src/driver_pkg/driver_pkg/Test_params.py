#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

class RobotParamSrv(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self.declare_parameter(name="sample_timer", value = 0.0, descriptor = ParameterDescriptor(description="Cinifiguracion del timer de muestreo"))
        self.declare_parameters(namespace='',parameters = [('lin_vel', 0.1),('ang_vel', 0.04), ("int_label", Parameter.Type.STRING)])
        self.add_on_set_parameters_callback(self._on_set_params_callback)
        self.get_logger().info("Param node working")

    def _on_set_params_callback(self, params:list[Parameter]):
        success_flag = True
        for param in params:
            if param.name == "sample_timer":
                if param.value < 0.0:
                    success_flag = False
                    self.get_logger().warning(f"el parametro {param.name} no puede ser negativo")
            else :
                self.get_logger().info(f"el parametro {param.name} no se monitorizado")

        result_msg = SetParametersResult()
        result_msg.reason = "Intercepted for validation"
        result_msg.successful = success_flag
        return result_msg

def init_node(): 
    rclpy.init()
    nodo = RobotParamSrv("x3p_params")
    rclpy.spin(nodo)
    rclpy.shutdown()

if __name__ == "__main__":
    init_node()