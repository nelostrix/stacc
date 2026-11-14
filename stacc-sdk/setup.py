from setuptools import setup, find_packages

setup(
    name="stacc",
    version="0.1.0",
    description="stacc — The Universal Robotics Development Platform",
    long_description=open("README.md").read() if __import__("os").path.exists("README.md") else "",
    long_description_content_type="text/markdown",
    author="stacc robotics",
    url="https://github.com/stacc-robotics/stacc",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=["numpy>=1.24.0", "PyYAML>=6.0"],
    extras_require={
        "test": ["pytest>=8.0", "scipy>=1.10"],
        "hal": ["pyserial>=3.5", "smbus2>=0.4.0"],
        "sim": ["mujoco>=3.0.0", "glfw>=2.6.0"],
        "server": ["fastapi>=0.110.0", "uvicorn>=0.29.0", "websockets>=12.0"],
        "deploy": ["paramiko>=3.0.0"],
        "manufacturing": ["trimesh>=4.0.0"],
        "all": [
            "mujoco>=3.0.0", "glfw>=2.6.0",
            "fastapi>=0.110.0", "uvicorn>=0.29.0", "websockets>=12.0",
            "paramiko>=3.0.0",
            "trimesh>=4.0.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "stacc=stacc.cli:main",
        ],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries :: Application Frameworks",
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
    ],
)
