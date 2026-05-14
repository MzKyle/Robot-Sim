from setuptools import setup

package_name = "data_collect_ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="huang",
    maintainer_email="huang@todo.todo",
    description="Desktop operator UI for weld data collection.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "data_collect_ui = data_collect_ui.app:main",
        ],
    },
)
