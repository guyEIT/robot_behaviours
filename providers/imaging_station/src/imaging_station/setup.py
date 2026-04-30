from setuptools import find_packages, setup

package_name = "imaging_station"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/imaging_station_sim.launch.py",
            "launch/imaging_station_ndi.launch.py",
            "launch/imaging_station_ndi_rviz.launch.py",
        ]),
        ("share/" + package_name + "/rviz", [
            "rviz/imaging_station_ndi.rviz",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Guy Burroughes",
    maintainer_email="guy.burroughes@eit.org",
    description=(
        "Generic plate imaging station provider with sim backend. "
        "Exposes the ImagePlate action consumed by campaign behaviour trees."
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "imaging_station_sim_node = imaging_station.sim_node:main",
            "imaging_station_ndi_node = imaging_station.ndi_node:main",
        ],
    },
)
