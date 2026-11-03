from glob import glob

from setuptools import find_packages, setup

package_name = 'craft_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pa29',
    maintainer_email='sssgggjjj4895@gmail.com',
    description='Voice narration: camera + planning state machine -> GPT (OpenAI) -> speech',
    license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['voice_node = craft_voice.voice_node:main',
                                      'voice_demo = craft_voice.demo:main']},
)
