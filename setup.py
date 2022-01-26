# Always prefer setuptools over distutils
import pathlib

from setuptools import find_packages, setup

here = pathlib.Path(__file__).parent.resolve()

# Get the long description from the README file
long_description = (here / "README.md").read_text(encoding="utf-8")

setup(
    name="pygraphslam",
    version="0.0.1",
    description="A python implementation of Graph SLAM",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/miquelmassot/pygraphslam",
    author="Miguel Massot",
    author_email="miquel.massot@gmail.com",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    python_requires=">=3.6, <4",
    entry_points={"console_scripts": ["pygraphslam=pygraphslam.pygraphslam:main"]},
    project_urls={
        "Bug Reports": "https://github.com/miquelmassot/pybpslam/issues",
        "Funding": "https://github.com/sponsors/miquelmassot",
        "Say Thanks!": "https://saythanks.io/to/miquelmassot",
        "Source": "https://github.com/miquelmassot/pybpslam/",
    },
)
