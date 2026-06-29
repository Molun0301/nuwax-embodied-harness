from setuptools import find_packages, setup

package_name = "nuwax_puppypi_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/cmd_vel_adapter.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="PuppyPi motion adapter for nuwax_robot_bridge",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_adapter = nuwax_puppypi_adapter.cmd_vel_adapter:main",
            "action_bridge = nuwax_puppypi_adapter.action_bridge:main",
        ],
    },
)
