from setuptools import find_packages, setup


package_name = "stereo_nav_tests"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="stereo_nav_sim",
    maintainer_email="maintainer@example.com",
    description="Acceptance checks for the pure-stereo simulation.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "runtime_audit = stereo_nav_tests.runtime_audit:main",
            "trajectory_evaluator = stereo_nav_tests.trajectory_evaluator:main",
            "navigation_acceptance = stereo_nav_tests.navigation_acceptance:main",
        ],
    },
)
