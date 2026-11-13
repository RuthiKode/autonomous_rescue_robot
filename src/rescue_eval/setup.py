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
        ],
    },
)
