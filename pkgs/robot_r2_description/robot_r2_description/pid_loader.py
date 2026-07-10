#!/usr/bin/env python3
"""Wait for plugin nodes then load PID YAML configs."""
import os
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def load_yaml(node, yaml_path, target_node_name):
    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    node_name = '/' + target_node_name.lstrip('/')
    if node_name not in data:
        return False

    params_dict = data[node_name]
    # Handle ros__parameters wrapper
    if 'ros__parameters' in params_dict:
        params_dict = params_dict['ros__parameters']

    client = node.create_client(
        'rcl_interfaces/srv/SetParameters',
        f'{node_name}/set_parameters')

    while not client.wait_for_service(timeout_sec=1.0):
        pass

    # Flatten nested dict to "ns.key" format
    flat_params = []
    def flatten(d, prefix=''):
        for k, v in d.items():
            if k in ('qos_overrides', 'use_sim_time'):
                continue
            full_key = f'{prefix}.{k}' if prefix else k
            if isinstance(v, dict):
                flatten(v, full_key)
            else:
                flat_params.append(Parameter(full_key, value=float(v)).to_parameter_msg())
    flatten(params_dict)

    from rcl_interfaces.srv import SetParameters
    req = SetParameters.Request()
    req.parameters = flat_params
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if future.result() is not None:
        for r in future.result().results:
            if not r.successful:
                return False
        return True
    return False


def main():
    rclpy.init()
    node = Node('pid_loader')

    pkg_dir = os.environ.get('ROBOT_R2_DESC_DIR', '')
    configs = [
        ('bar_lift_pid.yaml',   'robot_r2_bar_lift'),
        ('bar_rotate_pid.yaml', 'robot_r2_bar_rotate'),
        ('gripper_pid.yaml',    'robot_r2_gripper'),
    ]

    for filename, target in configs:
        yaml_path = os.path.join(pkg_dir, 'config', filename)
        if not os.path.exists(yaml_path):
            continue
        for _ in range(30):
            if load_yaml(node, yaml_path, target):
                break
            time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
