from setuptools import find_packages, setup

package_name = 'bucketDetector'

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
    maintainer='keshav',
    maintainer_email='mehndirattakeshav@gmail.com',
    description='Bucket detection ROS2 node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bucketDetector = bucketDetector.bucketDetector:main',  # Add this line
            'bucket = bucketDetector.bucket:main',  # Add this line
        ],
    },
)
