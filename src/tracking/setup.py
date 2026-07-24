from setuptools import find_packages, setup

package_name = 'tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evan',
    maintainer_email='evanfost@andrew.cmu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_emitter = tracking.test_emitter:main',
            'extrapolate_tracker = tracking.extrapolate_tracker:main',
            'constant_velocity_odom = tracking.constant_velocity_odom:main',
        ],
    },
)
