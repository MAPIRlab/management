from setuptools import setup

package_name = 'task_manager'

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
    maintainer='jgmonroy',
    maintainer_email='jgmonroy@uma.es',
    description='Behaviour Tree for managing high level robotic tasks',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_manager = task_manager.bt_manager:main'            
        ],
    },
)
