from setuptools import setup

package_name = 'go2_rtabmap'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mapping.launch.py']),
        ('share/' + package_name + '/launch', ['launch/camera_mapping.launch.py']),
        ('share/' + package_name + '/launch', ['launch/rtabmap_lidar.launch.py']),
        ('share/' + package_name + '/launch', ['launch/rtabmap_camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xk',
    maintainer_email='kexu985310@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
