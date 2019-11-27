from setuptools import setup

package_name = 'cozmo_ros2_nosdk'

setup(
    name=package_name,
    version='0.7.0',
    packages=[package_name],
    install_requires=['setuptools', 'numpy', 'pycozmo'],
    zip_safe=True,
    author='Alfonso Troya',
    author_email='alfonso.troya@gmail.com',
    maintainer='Alfonso Troya',
    maintainer_email='alfonso.troya@gmail.com',
    keywords=['ROS', 'Cozmo', 'driver'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Standalone ROS2 Driver for Cozmo (no SDK required)',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bringup = cozmo_ros2_nosdk.bringup:main',
            'teleop_twist_keyboard = cozmo_ros2_nosdk.teleop_twist_keyboard:main'
        ],
    },
)