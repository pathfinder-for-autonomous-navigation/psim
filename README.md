
# PSim

This repository holds all GNC code for the PAN mission. It will be comprised of
two main modules:

 1. GNC algorithms and utility functions implemented in C++ that will be
    used by flight software and our internal simulation - PSim.
 2. PSim models and simulation implementations.

MATLAB code listed in `MATLAB/**` is deprecated and will be phased out by the
new C++ implementation of PSim.

## Getting Started

For GNC development, it's only necessary to setup the PlatformIO development
environment. This requires setting up a local virtual environment as shown
below:

    python -m venv venv
    source venv/bin/activate
    pip install --upgrade pip
    pip install -r requirements.txt
    pio platform install native

Officially, we support Python 3.7 (this is what continuous integration runs) but
you'll most likely have success with Python 3.6+.

You can ensure everything is behaving as expected by building and running tests
for the GNC code with:

    source venv/bin/activate
    pio test -e native

**_Note: Naming your virtual environment `venv` is required for PSim
development!_**

For PSim development, we need to have the virtual environment setup as shown
above, a couple extra Python bells and whistles you most likely already have,
and then Bazel.

Besides your system install of Python and the virtual environment that will be
used to work with the PSim repository, you need to install the Python
development headers and `distutils` for the version of Python you'll be using.
The Bazel build system will require both of these to compile PSim into a Python
module.

Generally the Python development headers and `distutils` can be installed via a
package manager. Note that Bazel will explicitly let you know if it can't find
either of these so it may be worth just giving it a go before you hut down the
dependencies.

As far as Bazel is concerned, you should be able to install it through your
package manager of choice as well.

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
