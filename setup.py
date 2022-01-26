# Always prefer setuptools over distutils
import pathlib

from setuptools import find_packages, setup

here = pathlib.Path(__file__).parent.resolve()

# Get the long description from the README file
long_description = (here / "README.md").read_text(encoding="utf-8")

setup(
    name="pygraphslam",  # Required
    version="0.0.1",  # Required
    description="A python implementation of Graph SLAM",  # Optional
    long_description=long_description,  # Optional
    long_description_content_type="text/markdown",  # Optional (see note above)
    url="https://github.com/miquelmassot/pygraphslam",  # Optional
    author="Miguel Massot",  # Optional
    author_email="miquel.massot@gmail.com",  # Optional
    package_dir={"": "src"},  # Optional
    packages=find_packages(where="src"),  # Required
    python_requires=">=3.6, <4",
    entry_points={"console_scripts": ["pygraphslam=pygraphslam:main"]},  # Optional
    project_urls={  # Optional
        "Bug Reports": "https://github.com/miquelmassot/pybpslam/issues",
        "Funding": "https://github.com/sponsors/miquelmassot",
        "Say Thanks!": "https://saythanks.io/to/miquelmassot",
        "Source": "https://github.com/miquelmassot/pybpslam/",
    },
)
