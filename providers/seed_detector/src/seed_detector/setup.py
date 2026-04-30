import os
from setuptools import find_packages, setup

package_name = "seed_detector"

# Glob the models/ dir so a dropped-in last.pt ships into the install share
# without an extra setup.py edit. Pixi-build-ros's input globs include
# `models/**/*`, so changes here invalidate the build cache.
_models_dir = os.path.join(os.path.dirname(__file__) or ".", "models")
_model_files = [
    os.path.join("models", f)
    for f in sorted(os.listdir(_models_dir))
    if not f.startswith(".") and not f.endswith(".pyc")
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         ["launch/seed_detector.launch.py"]),
        ("share/" + package_name + "/models", _model_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Guy Burroughes",
    maintainer_email="guy.burroughes@eit.org",
    description=(
        "YOLO-based seed detector that consumes RealSense RGB+aligned-depth "
        "and publishes 3D detections in the camera optical frame."
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "seed_detector_node = seed_detector.detector_node:main",
        ],
    },
)
