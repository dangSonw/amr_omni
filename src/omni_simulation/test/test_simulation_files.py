import os
import unittest
import xml.etree.ElementTree as element_tree


class SimulationFilesTest(unittest.TestCase):
    def test_world_is_well_formed_sdf(self):
        root = os.path.dirname(os.path.dirname(__file__))
        element_tree.parse(os.path.join(root, 'worlds', 'amr_lab.sdf'))

    def test_simulation_config_and_bridge_are_present(self):
        root = os.path.dirname(os.path.dirname(__file__))
        for filename in ('config/simulation.yaml', 'config/gz_bridge.yaml'):
            self.assertTrue(os.path.isfile(os.path.join(root, filename)))

    def test_xacro_has_omni_joint_drive_configuration(self):
        root = os.path.dirname(os.path.dirname(__file__))
        path = os.path.join(
            os.path.dirname(root), 'omni_description', 'urdf', 'omni.urdf.xacro')
        with open(path) as stream:
            text = stream.read()
        self.assertNotIn('gz-sim-mecanum-drive-system', text)
        self.assertEqual(text.count('gz-sim-joint-controller-system'), 4)
        for wheel_index in ('1', '2', '3', '4'):
            self.assertIn('omni_wheel_joint_' + wheel_index, text)

    def test_headless_profile_uses_supported_lidar_sensor(self):
        root = os.path.dirname(os.path.dirname(__file__))
        launch_path = os.path.join(root, 'launch', 'simulation.launch.py')
        with open(launch_path) as stream:
            launch_text = stream.read()
        self.assertIn("' headless:=', headless", launch_text)

        description_root = os.path.join(
            os.path.dirname(root), 'omni_description')
        with open(os.path.join(description_root, 'urdf', 'omni.urdf.xacro')) as stream:
            robot_text = stream.read()
        with open(os.path.join(description_root, 'urdf', 'sensors.xacro')) as stream:
            sensor_text = stream.read()
        self.assertIn('<xacro:arg name="headless" default="false"/>', robot_text)
        self.assertEqual(sensor_text.count('type="gpu_lidar"'), 1)
        self.assertIn('type="gpu_lidar"', sensor_text)
        self.assertIn('<lidar><scan>', sensor_text)

    def test_robot_spawns_above_ground_for_upstream_wheel_geometry(self):
        root = os.path.dirname(os.path.dirname(__file__))
        path = os.path.join(root, 'launch', 'simulation.launch.py')
        with open(path) as stream:
            text = stream.read()
        self.assertIn("'-z', '0.1'", text)
        self.assertIn('--headless-rendering', text)
        with open(os.path.join(root, 'worlds', 'amr_lab.sdf')) as stream:
            self.assertIn('<mu>1.0</mu><mu2>1.0</mu2>', stream.read())

    def test_bridge_has_sensor_and_actuator_contracts(self):
        root = os.path.dirname(os.path.dirname(__file__))
        path = os.path.join(root, 'config', 'gz_bridge.yaml')
        with open(path) as stream:
            text = stream.read()
        for token in (
                'sensor_msgs/msg/LaserScan',
                'sensor_msgs/msg/Imu',
                'sensor_msgs/msg/Image',
                'sensor_msgs/msg/JointState',
                'std_msgs/msg/Float64'):
            self.assertIn(token, text)

    def test_simulation_uses_stm32_command_contract(self):
        root = os.path.dirname(os.path.dirname(__file__))
        path = os.path.join(root, 'config', 'simulation.yaml')
        with open(path) as stream:
            text = stream.read()
        self.assertIn('stm32_simulator:', text)
        self.assertIn('command_topic: stm32_cmd_vel', text)
        self.assertIn('control_frequency_hz: 100.0', text)
        self.assertIn('encoder_counts_per_revolution: 2048.0', text)
        self.assertIn('use_joint_encoder_input: true', text)
        self.assertIn('use_imu_input: true', text)
        self.assertIn('simulation_mode: true', text)

    def test_launch_runs_the_stm32_model_without_the_old_bypass_nodes(self):
        root = os.path.dirname(os.path.dirname(__file__))
        path = os.path.join(root, 'launch', 'simulation.launch.py')
        with open(path) as stream:
            text = stream.read()
        self.assertIn("executable='stm32_simulator'", text)
        self.assertNotIn('sim_wheel_command_bridge', text)
        self.assertNotIn("executable='sim_odometry'", text)

    def test_stm32_simulator_uses_simulated_wheel_joint_names(self):
        root = os.path.dirname(os.path.dirname(__file__))
        path = os.path.join(
            root, 'omni_simulation', 'stm32_simulator.py')
        with open(path) as stream:
            text = stream.read()
        self.assertIn('class Stm32Simulator', text)
        self.assertIn('forward_kinematics', text)
        self.assertIn('WheelSpeedPid', text)
        self.assertIn(
            'JointState, self.joint_states_topic, self._on_joint_state, 10,',
            text)
        self.assertIn('Imu, self.imu_input_topic, self._on_imu, 10, raw=True', text)
        self.assertIn('deserialize_message', text)
        self.assertIn('Invalid joint encoder message', text)
        self.assertIn('Int32MultiArray', text)
        self.assertIn('encoder_counts_publisher', text)

    def test_python_nodes_guard_shutdown_teardown(self):
        root = os.path.dirname(os.path.dirname(__file__))
        simulation_path = os.path.join(root, 'omni_simulation', 'stm32_simulator.py')
        hardware_path = os.path.join(
            os.path.dirname(root), 'omni_hardware', 'omni_hardware',
            'stm32_bridge.py')
        safety_path = os.path.join(
            os.path.dirname(root), 'omni_safety', 'omni_safety',
            'command_watchdog.py')
        with open(simulation_path) as stream:
            simulation_text = stream.read()
        with open(hardware_path) as stream:
            hardware_text = stream.read()
        with open(safety_path) as stream:
            safety_text = stream.read()
        for text in (simulation_text, hardware_text, safety_text):
            self.assertIn('except KeyboardInterrupt:', text)


if __name__ == '__main__':
    unittest.main()