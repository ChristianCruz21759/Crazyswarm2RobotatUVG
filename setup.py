from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'robotat'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Esto copia todo el contenido de launch/
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cruz',
    maintainer_email='cruz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
               	"testNode = robotat.testNode:main",
        	"goToRigidBody = robotat.goToRigidBody:main",
        	"followRigidBody = robotat.followRigidBody:main",
        	"goToOrigin = robotat.goToOrigin:main",
        	"goToInitialPosition = robotat.goToInitialPosition:main",
        	"hover = robotat.hover:main",
        	"graphPos = robotat.graph:main",
        	"latencyMeasure = robotat.latencyMeasure:main"
        ],
    },
)
