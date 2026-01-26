from glob import glob
from setuptools import setup, find_packages
from pybind11.setup_helpers import Pybind11Extension


ext_modules = [
    Pybind11Extension(
        "frc_ballistic_solver._core",
        sorted(glob("src/cpp/*.cpp", recursive=True))
    )
]

setup(
    name="frc_ballistic_solver",
    version="0.0.1",
    ext_modules=ext_modules,
    package_dir={'': 'src'},
    packages=find_packages(),
    include_package_data=True,
    package_data={
        "frc_ballistic_solver": ["*.pyi", "py.typed"]
    }
)
