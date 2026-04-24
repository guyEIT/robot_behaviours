from setuptools import find_packages, setup

package_name = "hamilton_star_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "transitions", "pylabrobot"],
    zip_safe=True,
    maintainer="Guy Burroughes",
    maintainer_email="guy.burroughes@eit.org",
    description="ROS 2 action server exposing a pylabrobot-driven Hamilton STAR.",
    license="Apache-2.0",
    extras_require={"test": ["pytest", "pytest-asyncio"]},
    entry_points={
        "console_scripts": [
            "action_server = hamilton_star_ros.action_server:main",
        ],
    },
)
