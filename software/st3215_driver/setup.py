from setuptools import setup, find_packages

setup(
    name="st3215_driver",
    version="0.1.0",
    description="Python library for controlling Waveshare ST3215 serial servos",
    long_description="A Python library for controlling Waveshare ST3215 serial servos with support for position, velocity, and PWM control modes.",
    author="Hnin Ei San",
    author_email="your.email@example.com",  # Update this
    url="https://github.com/thura-robotics/Hybrid_UAV-UGV",
    license="MIT",
    
    # Package discovery
    packages=find_packages(exclude=["test", "test.*"]),
    
    # Dependencies
    install_requires=["pyserial>=3.5"],
    python_requires=">=3.8",
    
    # Package metadata
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: System :: Hardware :: Hardware Drivers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    
    # Keywords for PyPI
    keywords="servo motor st3215 waveshare robotics hardware serial",
    
    # Entry points (optional - for command-line tools)
    # entry_points={
    #     'console_scripts': [
    #         'st3215-tool=st3215.cli:main',
    #     ],
    # },
)
