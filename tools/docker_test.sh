#!/usr/bin/env bash

# Starts a docker container and copies in your current local copy of PSim into
# the '/root/psim' directory. From there you can run the CI tests with the
# following command: 'platformio test -v -e ci_native'. This script needs to be
# called from the root PSim directory!
#

readonly id=$(docker create --rm -it psim:latest) &&
docker cp . "$id:/root/psim" &&
docker start -i "$id"

exit $?
