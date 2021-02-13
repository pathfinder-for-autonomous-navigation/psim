
# PSim

This repository holds all GNC code for the PAN mission. It will be comprised of
two main modules:

 1. GNC algorithms and utility functions implemented in C++ that will be
    used by flight software and our internal simulation - PSim.
 2. PSim models and simulation implementations.

MATLAB code listed in `MATLAB/**` is deprecated and will be phased out by the
new C++ implementation of PSim.

See the full set of documentation [here](https://pan-software.readthedocs.io/en/latest/psim/index.html)
including installation instructions.

## Getting Started

### GNC Development

Setting up the PlatformIO build system for GNC development can be done with the
following:

    python3 -m venv venv
    . venv/bin/activate
    pip install --upgrade pip
    pip install -r requirements.txt

You can ensure PlatformIO is behaving properly by running:

    . venv/bin/activate
    pio test -e native

**_Note: Using you're system's default Python 3 is recommended if you will also
be developing/working with PSim. PSim will build Python extensions using the
development headers associated with your system's default version of Python
3._**

### PSim development

For PSim development, there are the following additional requirements:

* The Bazel build system.
* Python 3 development headers installed on your system.
* Python 3 distutils package installed on your system.

All three of these should be available via your systems package manager. The
last build dependency (Python's distutls) is most likely installed on your
system already so it may be worth giving building a go and see if Bazel/Python
complain.

On a Debian based system, the Python development headers can be installed with:

    sudo apt-get install python3-dev

and Bazel install instructions (for other platforms too) can be found
[here](https://docs.bazel.build/versions/master/install-ubuntu.html#installing-bazel).

To ensure everything is working as expected, you should be able to build and run
the C++ unit tests for PSim with:

    bazel test //test/psim:all

The PSim Python module can be compiled, installed, and run with:

    python3 -m venv venv
    . venv/bin/activate
    pip install --upgrade pip
    pip install -r requirements.txt
    pip install -e .
    python -m psim --help

Note the you can add the `-vvv` flags to the `pip` command to see compilation
errors if the install fails. It's also recommended to use the `-e` flag to build
PSim in place which avoids making Bazel track a new repository that `pip` would
essentially create otherwise.

When making changes to PSim, simply run `pip install -e .` again to have Bazel
recompile the needed C++ files and `pip` update the Python source files.

### Bazel VSCode Extension and C/C++ Intellisense

It's also recommended to install the Bazel build tools along with the VSCode
extensions 'Bazel' by the 'Bazel Build Team'. With this extension plus
configuring C/C++ intellisense according to
`tools/bazel-compilation-database.sh` the Bazel build system should integrate
fairly well  with VSCode.

To ensure everything is working as expected, you should be able to run the
following successfully (make sure you're in your virtual environment when
working with Bazel as well):

    bazel test //test/psim:all

## GNC

The GNC library is a set of flight software algorithms (mainly controllers and
estimators) along with a myriad of utility functions.

The source code for the GNC library lives in `include/gnc/**` and `src/gnc/**`.
The associated unit tests are located in `test/gnc/**` and can be built with
PlatformIO using the following command:

    source venv/bin/activate
    pio test -e native

## PSim

The actual infrastructure for PSim is slightly fluid at the moment; however,
this PR have a good overview of the architecture: [#231](https://github.com/pathfinder-for-autonomous-navigation/psim/pull/231).

The code lives under `include/psim/**` and `src/psim/**` and leverages the GNC
modules as a dependency. Unit test are implemented under `test/psim/**` and can
be run with the following command:

    bazel test //test/psim:all --test_output=all

If you're interested in running the standalone version of PSim, you should install
a development version of the PSim module locally in you're virtual environment:

    source venv/bin/activate
    pip install -e .
    python -m psim --help

From there, you should be able to run PSim standalone cases, start simulation off
with difference configurations, and plot state fields over the course of the
simulation. Note, that `pip install -e .` calls the `setup.py` file which calls Bazel.
Therefore, if any C++ changes are made you must reinstall the PSim module with `pip`
to update the PSim binaries. Note the `-e` flag ensures `pip` doesn't copy the entire
respository and rebuild for each install.
