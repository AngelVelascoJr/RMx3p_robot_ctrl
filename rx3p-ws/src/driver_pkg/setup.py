from setuptools import find_packages, setup

package_name = 'driver_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Felipe Rivas',
    maintainer_email='rivascf@gmail.com',
    description='Paquete para el driver de la tarjeta ROSBoard.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'buzzer_listener = driver_pkg.Test_Buzzer:main',
            'gripper_listener = driver_pkg.gripper_ctrl:main',
            'test_params = driver_pkg.Test_params:init_node',
            'arm_params = driver_pkg.arm_ctrl_params:init_node',
            'arm_params_client = driver_pkg.arm_param_srv_cte:init_node'
        ],
    },
)
