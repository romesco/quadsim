from setuptools import find_packages
from setuptools import setup

requirements = [
    "pybullet",
    "numpy",
]

dev_requirements = [
    "pytest",
    "pytest-cov",
    "black",
    "pre-commit",
    "pre-commit-hooks",
    "flake8",
    "flake8-bugbear",
    "pep8-naming",
    "reorder-python-imports",
    "Pygments",
]

setup(
    name="quadsim",
    version="0.1.0",
    packages=find_packages(),
    author=["Yuxiang Yang, Rosario Scalise"],
    author_email=["yxyang@berkeley.edu", "rosario@cs.uw.edu"],
    url="http://github.com/romesco/quadsim",
    install_requires=requirements,
    extras_require={"dev": dev_requirements},
)
