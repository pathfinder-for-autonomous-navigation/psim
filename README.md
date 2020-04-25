
# PSim

This repository will hold all GNC code for the PAN mission. It will largely be
comprised of the two main executables/modules:

 1. MATLAB simulation to support GNC algorithm development.
 2. C++ versions of all flight GNC algorithms.

See folder specific READMEs for more information.

## Running CXX Tests Locally

Perform the following from the root PSim directory and replace `python` in the following commands with whatver command matches Python 3.7:

    python -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
    pio platform install native
    pio test -e native
