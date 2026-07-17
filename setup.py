from setuptools import setup
from glob import glob
from pathlib import Path

package_name = 'nav2_rl_project'


def files_only(pattern):
    """Return only regular files, excluding archive directories."""
    return [filename for filename in glob(pattern) if Path(filename).is_file()]

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/maps', files_only('maps/*')),
        ('share/' + package_name + '/paths', files_only('paths/*')),
        ('share/' + package_name + '/worlds', files_only('worlds/*')),
        ('share/' + package_name + '/config', files_only('config/*')),
        ('share/' + package_name + '/urdf', files_only('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mathiyalagan Vasantharaj, Isaac Vivin Moses',
    maintainer='Mathiyalagan Vasantharaj',
    maintainer_email='vasanmathi1999@gmail.com',
    description='Reinforcement-learning navigation for TurtleBot3 in Gazebo and on real hardware using saved Nav2 global paths.',
    license='MIT',
    scripts=[
        'scripts/save_global_path_from_rviz.py',
        'scripts/rviz_saved_path_overlay.py',
        'scripts/make_stage4_worlds.py',
    ],
)
