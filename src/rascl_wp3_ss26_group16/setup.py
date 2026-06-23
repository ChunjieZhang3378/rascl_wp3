from glob import glob
from setuptools import find_packages, setup


package_name = "rascl_wp3_ss26_group16"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where="src", exclude=["test"]),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        (
            "share/" + package_name + "/trajectories/input",
            glob("trajectories/input/*.csv"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Group 16",
    maintainer_email="TODO@example.com",
    description="ROS 2 package for RASCL Work Package 3 tasks.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wp3_tsk1 = rascl_wp3_ss26_group16.wp3_tsk1:main",
            "wp3_tsk2 = rascl_wp3_ss26_group16.wp3_tsk2:main",
        ],
    },
)
