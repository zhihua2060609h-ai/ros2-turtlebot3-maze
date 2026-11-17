from setuptools import setup
from glob import glob

setup(name='maze_control', version='0.1.0', packages=['maze_control'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/maze_control']),
                  ('share/maze_control', ['package.xml']),
                  ('share/maze_control/launch', glob('launch/*.py')),
                  ('share/maze_control/worlds', glob('worlds/*.world')),
                  ('share/maze_control/rviz', glob('rviz/*.rviz'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='hzh', maintainer_email='hzh@example.invalid', description='Course maze simulation',
      license='MIT', entry_points={'console_scripts': [
          'controller = maze_control.controller:main']})
