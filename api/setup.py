#!/usr/bin/python3

from setuptools import setup, find_packages

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name="hyshlr",
    version="1.0.0",
    author="Olivier Heriveaux",
    description="Python3 API for the Hayashi light remote control dongle",
    long_description=long_description,
    long_description_content_type="text/markdown",
    install_requires=["pyserial"],
    packages=find_packages(),
    python_requires=">=3.6")
