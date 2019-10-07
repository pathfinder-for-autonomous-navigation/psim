
# psim Python Interface

This folder will house all python code in this repository. Functionality to be
supported by Python may include, but is not limited to:

 * Running a simulation in real time with one or two satellites.
 * Real time graphing utilities and other data visulation tools to be used in
   conjunction with a real time simulation. See the following URLS:
   * https://stackoverflow.com/questions/11874767/how-do-i-plot-in-real-time-in-a-while-loop-using-matplotlib
   * https://learn.sparkfun.com/tutorials/graph-sensor-data-with-python-and-matplotlib/update-a-graph-in-real-time
 * Hardware/Sim interface for SHITLs.

# Installing

To install the Python MATLAB engine, you must:

 1. Install Python 3.6 - it's what we'll be using as it's the newest version
    officially supported by MATLAB.
 2. Activate a virtualenv here: `python3 -m virtualenv venv; source venv/bin/activate`
 3. Install the requirements: `pip install -r requirements.txt`
 4. I found that I required MATLAB R2019b in order for the simulation to work. Make sure you
    have this version.
 5. Set MATLAB's pyversion variable to link with the newly installed Python 3.6.
    You set and check the Python interpretter used by MATLAB by entering
    something like the following in MATLAB's terminal. Replace `PATH_TO_PSIM` with 
    the absolute path to this repository.

        >> pyversion PATH_TO_PSIM/python/venv/bin/python3
        >> pyversion

            version: '3.6'
            executable: '/Users/tanishqaggarwal/Documents/pan/repositories/psim/python/venv/bin/python3'
            library: ''
            home: '/Users/tanishqaggarwal/Documents/pan/repositories/psim/python/venv/bin/..'
            isloaded: 0

 6. Install MATLAB for this repository. On macOS:

         cd /Applications/MATLAB_R2019b.app/extern/engines/python
         python setup.py install --prefix="PATH_TO_PSIM/python/venv"

   The instructions are similar on other platforms.

# Running Simulation
Edit `usb_console/configs/fc_only.json` so that the `binary_filepath` for the Flight Controller points to a binary built
for the Flight Software. You can find these binaries [here](https://github.com/pathfinder-for-autonomous-navigation/FlightSoftware/releases).

Then, run the main script:

      python usb_console/run_simulation.py -c usb_console/configs/fc_only_native.json

This starts up the simulation and uses a desktop binary for the FlightSoftware.

If you'd like to run more complex configurations, see the examples in the folder `usb_console/configs`.
