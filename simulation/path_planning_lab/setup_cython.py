"""Build the optional DWA Cython hot loop in place.

Usage (from ``simulation/path_planning_lab``):

    python -m pip install "Cython>=3.1,<4"
    python setup_cython.py build_ext --inplace
"""

import numpy as np
from Cython.Build import cythonize
from setuptools import Extension, find_packages, setup

setup(
    name="hospital-path-planning-lab-dwa-hotloop",
    package_dir={"": "src"},
    packages=find_packages("src"),
    ext_modules=cythonize(
        [
            Extension(
                "hospital_path_lab._dwa_hotloop",
                ["src/hospital_path_lab/_dwa_hotloop.pyx"],
                include_dirs=[np.get_include()],
            )
        ],
        compiler_directives={"language_level": "3"},
    ),
)
