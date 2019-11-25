#!/usr/bin/env bash

# Builds the docker container used for Ubuntu gcc testing. This script needs to
# be called from the root PSim directory!
#

docker image build -t psim:latest .

exit $?
