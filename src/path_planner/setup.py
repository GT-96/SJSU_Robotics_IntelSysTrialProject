from setuptools import find_packages, setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tegh',
    maintainer_email='Gill.Teghbir@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map = path_planner.map:main',
            'goal_and_start = path_planner.goal_and_start:main',
            'path = path_planner.path:main',
            'viz = path_planner.viz:main'
        ],
    },
)
