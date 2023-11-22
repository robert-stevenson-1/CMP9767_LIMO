from setuptools import find_packages, setup

package_name = 'workshop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robert',
    maintainer_email='78357805+robert-stevenson-1@users.noreply.github.com',
    description='Package containing the workshop code/files',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opencv_test = workshop.opencv_test:main',
        ],
    },
)
