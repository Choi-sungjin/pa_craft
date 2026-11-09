from glob import glob

from setuptools import setup

package_name = 'craft_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'sim.sh']),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SungjinChoi',
    maintainer_email='208406003+Choi-sungjin@users.noreply.github.com',
    description='2D planning simulator for craft (real planning/control masters, fake wheel board, ground-truth perception)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run = craft_sim.run_scenario:main',
            'sim_node = craft_sim.sim_node:main',
            'summary = craft_sim.summary:main',
        ],
    },
)
