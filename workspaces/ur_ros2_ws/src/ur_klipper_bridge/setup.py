from setuptools import find_packages, setup


package_name = "ur_klipper_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yi",
    maintainer_email="yi@example.com",
    description="Persistent ROS 2 bridge that owns guarded Moonraker writes.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "klipper_bridge = ur_klipper_bridge.bridge_node:main",
        ],
    },
)
