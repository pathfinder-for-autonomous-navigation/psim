
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
 2. Set MATLAB's pyversion variable to link with the newly installed Python 3.6.
    You set and check the Python interpretter used by MATLAB by entering
    something like the following in MATLAB's terminal where you'd replace
    `/opt/local/bin/python3` with the path to your Python 3.6 interpretter:

        >> pyversion /opt/local/bin/python3
        >> pyversion

            version: '3.6'
            executable: '/opt/local/bin/python3'
            library: '/opt/local/Library/Frameworks/Python.framework/Versions/3.6/lib/libpython3.6m.dylib'
            home: '/opt/local/Library/Frameworks/Python.framework/Versions/3.6'
            isloaded: 0
        
        >>

  3. Run the install script for the MATLAB engine library. Follow the
     instructions posted at the following URL:  
     https://www.mathworks.com/help/matlab/matlab_external/install-the-matlab-engine-for-python.html  
     Note: On mac, I recommend running the command from terminal yourself and
     not the MATLAB terminal. Make sure to substitue `python3` for `python` if
     you didn't change your default Python version.

# Running Simulation

  1. Activate a virtualenv: `python3 -m virtualenv venv; source venv/bin/activate`
  2. Install the requirements: `pip install -r requirements.txt`
  3. Move into the usb_console directory.
  4. Run the main script: `python usb_console/run_simulation.py -c usb_console/configs/fc_only.json`
