from setuptools import setup
from glob import glob

package_name = 'boring_nodes'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
     ('share/' + package_name+ '/parameter_files', glob('parameter_files/*')),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Nate Bunderson',
 maintainer_email='nate.bunderson@asirobots.com',
 description='Nodes for a fairly boring ackermann robot node',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'boring_planner = boring_nodes.boring_planner:main',
             'boring_grid = boring_nodes.boring_grid:main',
             'boring_controller = boring_nodes.boring_controller:main',
             'boring_ackermann = boring_nodes.boring_ackermann:main'
     ],
   },
)