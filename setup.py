from setuptools import setup
from glob import glob

package_name = 'nav2_rl_project'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/paths', glob('paths/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mathiyalagan Vasantharaj',
    maintainer_email='vasanmathi1999@gmail.com',
    description='Reinforcement learning local controller for TurtleBot3 navigation using a saved Nav2 global path.',
    license='MIT',
    scripts=[
        'scripts/save_global_path_from_rviz.py',
        'scripts/rviz_saved_path_overlay.py',
        'scripts/make_stage4_worlds.py',
    ],
)
