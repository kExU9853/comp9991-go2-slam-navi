from setuptools import setup
import os
from glob import glob

package_name = 'go2_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # 添加这一行！
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xk',
    maintainer_email='kexu985310@gmail.com',
    description='Launch package for Nav2 on Go2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

