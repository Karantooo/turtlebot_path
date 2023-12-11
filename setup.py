from setuptools import setup

package_name = 'turtlebot_path'

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
    maintainer='karantooo',
    maintainer_email='karantooo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "draw_circle = turtlebot_path.draw_circle:main",
            "image_subscriber = turtlebot_path.image_subscriber:main"
        ],
    },
)
