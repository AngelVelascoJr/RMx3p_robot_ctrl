#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.srv import ListParameters, GetParameters

class ArmParamCte(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._param_names = []
        self._params_dict = {}

        #Declaracion de la llamada a servicios
        self._get_param_client = self.create_client(
            GetParameters,
            "/the_arm_param_srv/get_parameters"
        )
        self._list_param_client = self.create_client(
            ListParameters, "/the_arm_param_srv/list_parameters"
        )
        list_param_request = ListParameters.Request()
        list_param_request.prefixes.append('arm')
        list_param_request.depth = 0
        self.get_logger().info(f"calling /arm_param_srv/list_parameters: ListParameter.Request: prefixes:{list_param_request.prefixes}, depth:{list_param_request.depth}")

        try: 
            tries = 3
            while not self._list_param_client.wait_for_service(timeout_sec=1):
                self.get_logger().info(f"Service '/arm_param_srv/list_parameters' not available ({tries}), trying again")
                tries -= 1
                if tries <= 0:
                    self.get_logger().warning(f"Service '/arm_param_srv/list_parameters' not available")
                    return
            self.get_logger().info("Calling '/the_arm_param_srv/list_parameters' async")

            future = self._list_param_client.call_async(list_param_request)
            future.add_done_callback(self.__on_list_param_srv_callback)

        except Exception as e:
            self.get_logger().warning("Service call failed %r " % (e,))

    def __on_list_param_srv_callback(self, future):
        try:
            list_params_result = future.result().result#ListParameters.Response()
            if list_params_result.names:
                get_parameters_request = GetParameters.Request()
                self._param_names = get_parameters_request.names = list_params_result.names
                tries = 3
                while not self._get_param_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f"Service '/the_arm_param_srv/list_parameters'not available ({tries}), trying again")
                    tries -= 1
                    if tries <= 0:
                        self.get_logger().warning(f"Service not available")
                        return
                self.get_logger().info("Calling '/the_arm_param_srv/list_parameters' servie async")
                future = self._get_param_client.call_async(get_parameters_request)
                future.add_done_callback(self._on_get_param_srv_callback)

        except Exception as e:
            self.get_logger().warning("On service callback failed %r " % (e,))

    def _on_get_param_srv_callback(self, future):
        try:
            get_parameters_result = future.result()
            arm_params_dict = self.__get_recursive_params(self._param_names, get_parameters_result.values,'arm.')
            print(arm_params_dict)            
            self._params_dict = arm_params_dict
        except Exception as e:
            self.get_logger().warning("On service call failed %r " % (e,))

    def __get_recursive_params(self, names, values, prefix):
        result = {}
        index = 0
        for param_name in names:
            if param_name.startswith(prefix):
                key_path = param_name[len(prefix):].strip('.')
                keys = key_path.split('.')
                current = result
                for k in keys[:-1]:
                    current = current.setdefault(k, {})
                current[keys[-1]] = self.__parse_parameter_value(values[index])
            index += 1

        return result

    def __parse_parameter_value(self, parameter: ParameterValue):
        if parameter.type == ParameterType.PARAMETER_BOOL:
            return parameter.bool_value
        elif parameter.type == ParameterType.PARAMETER_INTEGER:
            return parameter.integer_value
        elif parameter.type == ParameterType.PARAMETER_DOUBLE:
            return parameter.double_value
        elif parameter.type == ParameterType.PARAMETER_STRING:
            return parameter.string_value
        elif parameter.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return parameter.byte_array_value
        elif parameter.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return parameter.bool_array_value
        elif parameter.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return parameter.integer_array_value.tolist()
        elif parameter.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return parameter.double_array_value.tolist()
        elif parameter.type == ParameterType.PARAMETER_STRING_ARRAY:
            return parameter.string_array_value
        else:
            return None

def init_node(args = None):
    try:
        rclpy.init(args=args)
        arm_param_client_node = ArmParamCte("the_arm_param_client")
        rclpy.spin(arm_param_client_node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("program finalized.")
    except Exception as e:
        print(e)    

if __name__ == "__main__":
    init_node()