from setuptools import setup

package_name = 'my_camera_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'my_camera_package.image2',
        'my_camera_package.occupancy_grid_map_node',  # Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Description of the package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image2 = my_camera_package.image2:main',
            'occupancy_grid_map_node = my_camera_package.occupancy_grid_map_node:main',  # Add this line
        ],
    },
)
