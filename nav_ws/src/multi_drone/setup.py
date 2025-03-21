from setuptools import find_packages, setup

package_name = 'multi_drone'

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
    maintainer='em',
    maintainer_email='22knc5@queensu.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_takeoff_1 = multi_drone.multi_takeoff_1:main',
            'multi_takeoff_2 = multi_drone.multi_takeoff_2:main',
            'multi_takeoff_3 = multi_drone.multi_takeoff_3:main',
            'multi_takeoff_4 = multi_drone.multi_takeoff_4:main',
            'multi_takeoff_5 = multi_drone.multi_takeoff_5:main',
            'multi_takeoff_6 = multi_drone.multi_takeoff_6:main',
            'multi_takeoff_7 = multi_drone.multi_takeoff_7:main',
            'multi_takeoff_8 = multi_drone.multi_takeoff_8:main'
        ],
    },
)
