from setuptools import find_packages, setup

package_name = 'projection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evan',
    maintainer_email='evanfost@andrew.cmu.edu',
    description='Handles projecting detection boxes into 3D space based on depth information',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': ['basic_projection = projection.basic_projection:main'],
    },
)
