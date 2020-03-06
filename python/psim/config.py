import json
import os
import sys


def _directories_generator(base_directory, directory):
    """Streams the directories obtained from walking up the configuration tree
    starting from dir and ending on base_dir (inclusive).
    """
    # Ensure the base directory exists
    if not os.path.isdir(base_directory):
        raise RuntimeError('Invalid base configuration directory: ' + base_directory)

    # Ensure the specified directory is valid
    directory = os.path.abspath(os.path.expanduser(directory))
    if not os.path.isdir(directory) or not directory.startswith(base_directory):
        raise RuntimeError('Invalid configuration directory:' + directory)

    yield directory
    while directory is not base_directory:
        directory = os.path.dirname(directory)
        yield directory


def load(directory):
    """
    returns:
        leader, follower
    """
    base_directory = os.path.join(os.path.dirname(os.path.abspath(os.path.expanduser(__file__))), 'config')

    # Iterate through directories
    print(base_directory)
    for dir in _directories_generator(base_directory, directory):
        print(directory)


load('psim/config/deployment')
