from setuptools import setup
from glob import glob
import os

package_name = 'mazebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name,'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name,'world'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luqman',
    maintainer_email='noshluk2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driving_node = mazebot.driving:main' ,
            'go_to_goal = mazebot.go_to_goal:main',
            'video_recorder = mazebot.video_saver:main',
            'maze_solver = mazebot.maze_solver:main',
        ],
    },
)