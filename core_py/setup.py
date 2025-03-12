from setuptools import find_packages, setup

package_name = 'core_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/ReloadParams.srv', 'srv/StartSensor.srv', 'srv/StopSensor.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pe65',
    maintainer_email='pe65@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lifecycle = core_py.lifecycle:main',
        ],
    },
)
