from setuptools import find_packages, setup

package_name = 'moveMotor'

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
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PublisAndMoveJoystick=moveMotor.joystickTest:main',
            'MotorTopic=moveMotor.moveMotorTopic:main',
            'Habla=moveMotor.hablando:main',
            'Escucha=moveMotor.escuchando:main',
            'Qcamera=moveMotor.reader:main',
            'Mensaje=moveMotor.mensajes:main',
        ],
    },
)
