
from setuptools import setup, find_packages

setup(
    name="differential_yaw_moment",
    version="0.1.0",
    packages=["vehicle_dynamics"],
    install_requires=[
        "numpy",
        "matplotlib",
        "scipy",
    ],
    author="Gemini",
    author_email="",
    description="A vehicle dynamics model for analyzing yaw moment and handling balance.",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
