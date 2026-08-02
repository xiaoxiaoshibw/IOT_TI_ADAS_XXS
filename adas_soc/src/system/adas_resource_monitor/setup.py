from setuptools import find_packages, setup


package_name = "adas_resource_monitor"

setup(
    name=package_name,
    version="0.0.2",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="adas",
    maintainer_email="dev@adas.local",
    description="Orin resource and platform health diagnostics.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "resource_monitor = adas_resource_monitor.node:main",
    ]},
)
