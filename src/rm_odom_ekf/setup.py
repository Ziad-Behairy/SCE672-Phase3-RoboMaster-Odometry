from glob import glob

from setuptools import setup

package_name = 'rm_odom_ekf'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ziad Aboelkhir',
    maintainer_email='zezomoh951@gmail.com',
    description='Odometry prediction + camera tag correction (5-state EKF) for the RoboMaster EP (SCE 672 Phase 3).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_detector = rm_odom_ekf.tag_detector:main',
            'phase3_ekf = rm_odom_ekf.phase3_node:main',
        ],
    },
)
