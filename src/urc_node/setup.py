from setuptools import setup, find_packages   # find_packages 사용
from glob import glob
import os

package_name = 'urc_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),  # 자동 탐색 (오타/경로 실수 방지)
    data_files=[
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),  # resource/urc_node 파일 꼭 존재!
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],     # 시리얼 쓰니 명시 권장
    zip_safe=True,
    maintainer='cyj',
    maintainer_email='mingu3830@gmail.com',
    description='URC robot control node package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'urc_drive_controller = urc_node.urc_drive_controller:main',
        ],
    },
)

