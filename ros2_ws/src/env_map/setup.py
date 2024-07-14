from setuptools import setup

package_name = 'env_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','opencv-python', 'numpy', 'cv_bridge'],
    zip_safe=True,
    maintainer='sagar',
    maintainer_email='sagar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['object_detection_node = your_package_name.object_detection_node:main',
        ],
    },
)
