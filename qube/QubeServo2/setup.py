from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize

extensions = [
    Extension("qube", ["qube.pyx"],
        include_dirs = ["/opt/quanser/hil_sdk/include"],
        libraries = ["hil", "quanser_runtime", "quanser_common", "rt", "pthread", "dl", "m", "c"],
        library_dirs = ["/opt/quanser/hil_sdk/lib"],
    )
]
setup(ext_modules=cythonize(extensions))
