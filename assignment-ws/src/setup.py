from setuptools import find_packages, setup

package_name = 'assignment'

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
    maintainer='robert',
    maintainer_email='78357805+robert-stevenson-1@users.noreply.github.com',
    description='Package containing robot programming assignment',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = assignment.navigator:main',
            'example_nav_to_pose = assignment_template.example_nav_to_pose:main',
            'object_detector = assignment_template.object_detector:main',
        ],
    },
)
