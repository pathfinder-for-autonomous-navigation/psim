The main gravity function, `geograv::GeoGrav` and the definition of the `geograv::Coeff` class is in `include/geograv.hpp`

`include/EGM96.hpp` has the coefficients for the 360x360 egm96 model.

`teensy_test` is an arduino sketch to run on the teensy.

`gravmodelgen.py` is a python script to convert text files with coefficients to the c++ header files with coefficients.

`main.cpp` is a test file for running on desktop.
## Installation and Running Tests

First install [geographiclib](https://geographiclib.sourceforge.io/html/index.html):
On mac you can just

`brew install geographiclib`

Then run

`/usr/local/sbin/geographiclib-get-gravity all`

 to install all of the gravity models.

 To run the tests,
First open terminal in the `geograv` directory, then run the following commands:

`mkdir build`

`cd build`

`cmake ..`

`make`

`./TestGeoGrav`

To generate new coefficient header files, use `gravmodelgen.py`, for example:

`python gravmodelgen.py -f models/egm96 -n 360 -o include/EGM96.hpp -r 6378137.0 -u 0.3986004418e15`

and

`python gravmodelgen.py -f models/egm96 -n 20 -o include/EGM96_20.hpp -r 6378137.0 -u 0.3986004418e15`
