from setuptools import setup

package_name = 'vi_grab'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='fishros@todo.todo',
    description='YOLOv8 + RealSense publisher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_rs_publisher = vi_grab.yolo_rs_publisher:main',
            'grasp_executor = vi_grab.grasp_executor:main',
            'camera_to_base = vi_grab.camera_to_base:main',
        ],
    },
)

