from glob import glob

from setuptools import find_packages, setup

package_name = 'rescue_eval'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/drives', glob('drives/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ruthik',
    maintainer_email='ruthik@todo.todo',
    description='Automated run-and-score harness for the rescue robot.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'run_eval = rescue_eval.run_eval:main',
            'sensor_check = rescue_eval.sensor_check:main',
            'odom_check = rescue_eval.odom_check:main',
            'camera_rate_check = rescue_eval.camera_rate_check:main',
            'ekf_validation = rescue_eval.ekf_validation:main',
            'cmd_vel_record = rescue_eval.cmd_vel_io:record_main',
            'cmd_vel_replay = rescue_eval.cmd_vel_io:replay_main',
            'auto_drive = rescue_eval.auto_drive:main',
            'tf_audit = rescue_eval.tf_audit:main',
            'map_score = rescue_eval.map_score:main',
            'summarize = rescue_eval.summarize:main',
            'rescore = rescue_eval.rescore:main',
            'nav_goals = rescue_eval.nav_goals:main',
            'explore_eval = rescue_eval.explore_eval:main',
        ],
    },
)
