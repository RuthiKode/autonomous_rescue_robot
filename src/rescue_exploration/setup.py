from glob import glob

from setuptools import find_packages, setup

package_name = 'rescue_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ruthik',
    maintainer_email='ruthik@todo.todo',
    description='Frontier exploration: explore_lite configuration and the own frontier selector.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'frontier_selector = rescue_exploration.selector_node:main',
        'map_stats = rescue_exploration.map_stats_node:main',
    ]},
)
