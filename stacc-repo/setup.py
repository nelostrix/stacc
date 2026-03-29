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
    python_requires=">=3.8",
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
