from setuptools import setup
package_name = 'asi_ctrl'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jacob Knauo',
    maintainer_email='jacobk@gatech.edu',
    description='Controller developed by GT for controlling ASI vehicle',
    license='BSD',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cssmpc = asi_ctrl.cs_controller_asi_convex:main',
            'map_ca = asi_ctrl.polylines_asi:main'
        ],
    },
)
