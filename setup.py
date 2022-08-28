from setuptools import setup

package_name = 'tello_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pop',
    maintainer_email='sahar45456@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'extreme = tello_controller.ExtremeSubscriber:main',
            'object_track_xbox = tello_controller.XboxSubscriber:main'
        ],
    },
)
