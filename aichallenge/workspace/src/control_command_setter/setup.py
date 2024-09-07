from setuptools import setup
import glob
import os

package_name = 'control_command_setter'
setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('./model/*.onnx'))
    ],
    install_requires=['setuptools', 'pygame', 'onnxruntime'],
    zip_safe=True,
    maintainer='bushio',
    maintainer_email='bushio@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_setter = control_command_setter.command_setter:main',
            'controller_cmd_setter = control_command_setter.controller_cmd_setter:main',
            'control_cmd_predictor = control_command_setter.control_cmd_predictor:main',
        ],
    },
)
