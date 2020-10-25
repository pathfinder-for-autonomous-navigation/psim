# Inspired by:
#  - https://github.com/google/tensorstore/blob/master/setup.py
#  - https://github.com/google/tensorstore/blob/master/bazelisk.py
#  - https://stackoverflow.com/questions/38125588/how-to-write-setup-py-to-install-python-extention-xxx-so-file-built-by-swig

import setuptools
import setuptools.command.build_ext

import os
import shutil
import subprocess

__version__ = '0.0.1'


class BuildExtCommand(setuptools.command.build_ext.build_ext):

    def run(self):
        p = subprocess.Popen(['bazel', 'build', '//:_psim'])
        while True:
            try:
                p.wait()
                break
            except KeyboardInterrupt:
                # Bazel will also get the signal and terminate.
                # We should continue waiting until it does so.
                pass

        ext = self.extensions[0]
        shutil.copyfile('bazel-bin/_psim.so', self.get_ext_fullpath(ext.name))


if os.name == 'Windows':
    raise RuntimeError('Installing psim with pip is not compatible with Windows')

setuptools.setup(
    name = 'psim',
    version = __version__,
    author = 'Kyle Krol',
    description = '6-DOF simulation for the PAN missions',
    install_requires=['numpy', 'pyyaml', 'matplotlib'],
    ext_modules=[setuptools.Extension('_psim', sources=[])],
    cmdclass = {'build_ext': BuildExtCommand},
    packages = ['psim', 'psim.plugins'],
    package_dir = {'': 'python'},
    #package_data = {'': ['_psim.so']},
)
