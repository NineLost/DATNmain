from setuptools import setup

package_name = 'dual_lidar_tools'

setup(
    name=package_name,          # KHÔNG dùng dấu '-' ở đây
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lota',
    maintainer_email='lota@example.com',
    description='Tools for merging two lidar scans',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # console_script: tên lệnh  = module:function
            'dual_lidar_merger = dual_lidar_tools.dual_lidar_merger:main',
        ],
    },
)

