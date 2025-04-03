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
        (os.path.join('share',package_name,'urdf'),glob('urdf/*')),
        (os.path.join('share',package_name,'meshes'),glob('meshes/*')),
        (os.path.join('share',package_name,'launch'),glob('launch/*')),
        (os.path.join('share',package_name,'world'),glob('world/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niran',
    maintainer_email='nirans2002@gmail.com',
    description='maze bot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'talker = mazebot.publisher_member_function:main',
             'listener = mazebot.subscriber_member_function:main',
             'drive_maze_bot = mazebot.driving_node:main',
             'go_to_goal = mazebot.go_to_goal:main',
             'video_saver_node= mazebot.video_saver:main',
             'maze_solver= mazebot.maze_solver:main',
        ],
    },
)
