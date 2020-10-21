# Inspired by:
#  - https://github.com/google/tensorstore/blob/master/setup.py
#  - https://github.com/google/tensorstore/blob/master/bazelisk.py
#  - https://stackoverflow.com/questions/38125588/how-to-write-setup-py-to-install-python-extention-xxx-so-file-built-by-swig

import setuptools
import setuptools.command.build_ext

import os
import shutil
import subprocess
import sys

__version__ = '0.0.1'


def bazel(argv):
  p = subprocess.Popen(['bazel'] + argv)
  while True:
    try:
      return p.wait()
    except KeyboardInterrupt:
      # Bazel will also get the signal and terminate.
      # We should continue waiting until it does so.
      pass


if os.name == 'Windows':
    raise RuntimeError('Installing psim with pip is not compatible with Windows')

# Build the bazel Python extension
bazel(['build', '//:_psim'])
shutil.copyfile('bazel-bin/python/_psim.so', '_psim.so')

setuptools.setup(
    name = 'psim',
    version = __version__,
    author = 'Kyle Krol',
    description = '6-DOF simulation for the PAN missions',
    py_modules = ['_psim'],
    packages = ['psim', 'psim.plugins'],
    package_data = {'': ['_psim.so']},
)
