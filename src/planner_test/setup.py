from setuptools import setup, find_packages

package_name = 'planner_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # <--- KLUCZOWE
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mateusz',
    description='Testowanie plannerów Nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_planner = planner_test.test_planner:main',
            'odom_to_path_publisher = planner_test.odom_to_path_publisher:main'
        ],
    },
)
