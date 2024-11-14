from setuptools import find_packages, setup

package_name = 'pallets_detection'

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
    maintainer='minato',
    maintainer_email='vybhavraghav@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection = pallets_detection.detection:main',
            'camera = pallets_detection.camera_node:main',
            'video = pallets_detection.video:main',

        ],
    },
)
