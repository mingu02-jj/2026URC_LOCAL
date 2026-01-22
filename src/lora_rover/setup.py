from setuptools import setup

package_name = 'lora_rover'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rover_lora.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='URC Team',
    maintainer_email='user@example.com',
    description='URC LoRa(E22) rover RX: E22 serial frames -> /cmd_vel_lora (+ /estop_lora)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lora_rover_rx = lora_rover.lora_rover_rx:main',
        ],
    },
)
