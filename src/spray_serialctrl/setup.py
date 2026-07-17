from setuptools import find_packages, setup

package_name = 'spray_serialctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chloe Zhang',
    maintainer_email='cczhang@andrew.cmu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dispatcher = spray_serialctrl.dispatcher:main',
            'serial_controller = spray_serialctrl.serialcontroller:main',
        ],
    },
)
