from setuptools import setup

package_name = 'orb_slam_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kensei Nakamura',
    maintainer_email='kensei@artifarm.com',
    description='ORB SLAM3 ros utils',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mono_scale = orb_slam_utils.mono_scale:main',
        ],
    },
)
