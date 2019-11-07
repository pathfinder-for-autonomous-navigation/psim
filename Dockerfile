# Dockerfile for running PlatformIO unit tests in a Linux environment. This is
# specifically to allow Mac developers to troubleshoot potential precision
# issues as seen in the past.
#

FROM ubuntu:18.04

RUN \
  apt-get update && \
  apt-get -y upgrade && \
  apt-get install -y \
    python3 \
    python3-pip

RUN \
  python3 -m pip install \
    platformio

# Following two environment variables were required for PIO
ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN \
  platformio update
