import os
from glob import glob
from setuptools import setup

package_name = 'lpl_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- ADDED LINES START ---
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # --- ADDED LINES END ---
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='LPL Demo Package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lpl_manager = lpl_demo.lpl_manager:main',
            'lpl_manager_formula = lpl_demo.lpl_manager_formula:main',
            'human_authorize = lpl_demo.human_authorize:main',
            'lpl_dashboard = lpl_demo.lpl_dashboard:main',
        ],
    },
)   