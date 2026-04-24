from setuptools import find_packages, setup

package_name = "liconic_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/liconic.launch.py"]),
    ],
    install_requires=["setuptools", "pylabrobot", "pyserial"],
    zip_safe=True,
    maintainer="Guy Burroughes",
    maintainer_email="guy.burroughes@eit.org",
    description="ROS 2 action server exposing a pylabrobot-driven Liconic STX incubator.",
    license="Apache-2.0",
    extras_require={"test": ["pytest", "pytest-asyncio"]},
    entry_points={
        "console_scripts": [
            "action_server = liconic_ros.action_server:main",
        ],
    },
)
