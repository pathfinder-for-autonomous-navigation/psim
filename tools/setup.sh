# used to setup a psim venv, and also used to build python psim module
sudo apt-get install python3-dev
rm -r venv
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
bazel build //test/psim:all
bazel run //python:main -- --help