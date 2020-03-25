import json
import logging
import os
import sys

_log = logging.getLogger(__name__)

_base_directory = os.path.join(os.path.dirname(__file__), 'config')

global_modules = { 'constants' }
satellite_modules = { 'sensors', 'truth' }
all_modules = global_modules.union(satellite_modules)


def _get_full_path(path):
    """Expands the user home directory and any symbollic links.

    Args:
        path:

    Returns:
        Fully expanded version of the path.
    """
    return os.path.abspath(os.path.expanduser(path))


def _get_config_directories(directory, base_directory):
    """Gives a list of the directories ecountered while walking down the
    configuration tree.

    Args:
        directory:
        base_directory:

    Returns:
        List of directories.
    """
    # Ensure the base directory exists
    base_directory = _get_full_path(base_directory)
    if not os.path.isdir(base_directory):
        raise RuntimeError('Invalid base configuration directory: %s' % base_directory)

    # Ensure the specified directory is valid
    directory = _get_full_path(os.path.join(base_directory, directory))
    if not os.path.isdir(directory) or not directory.startswith(base_directory):
        raise RuntimeError('Invalid configuration directory: %s' % directory)

    directories = list()
    while directory.startswith(base_directory):
        directories.append(directory)
        directory = _get_full_path(os.path.dirname(directory))

    directories.reverse()
    return directories


def _get_module_dictionaries(module, directory, base_directory):
    """Streams the json dictionaries for the desired module encountered while
    walking down the configuration tree.

    Args:
        directory:
        base_directory:
        module:
    """
    for _directory in _get_config_directories(directory, base_directory):
        potential_json = os.path.join(_directory, module + ".json")
        if os.path.isfile(potential_json):
            with open(potential_json, 'r') as _json:
                yield json.load(_json)


def _load_module(module, directory, base_directory=_base_directory):
    """
    Args:
        module:
        directory:
        base_directory:

    Returns:
    """
    def _search_for_sub_dictionary(dictionary, sub_dictionary):
        _sub_dictionary = dict()
        if (sub_dictionary in dictionary.keys()):
            _sub_dictionary = dictionary[sub_dictionary]
            if not (type(_sub_dictionary) is dict):
                raise RuntimeError('"%s" entry is expected to be a dictionary' % sub_dictionary)

        return _sub_dictionary

    leader = dict()
    follower = dict()
    for dictionary in _get_module_dictionaries(module, directory, _base_directory):

        # Look for shared, leader, and follower entries
        _shared = _search_for_sub_dictionary(dictionary, 'shared')
        _leader = _search_for_sub_dictionary(dictionary, 'leader')
        _follower = _search_for_sub_dictionary(dictionary, 'follower')

        # Prepare the new leader dictionary
        leader.update(_shared)
        leader.update(_leader)

        # Prepare the new follower dictionary
        follower.update(_shared)
        follower.update(_follower)
    
    return leader, follower


def load_modules(modules, config_dir, patch_dir=None):
    """
    """
    return dict()


def load_all_modules(modules, config_dir, patch_dir=None):
    """
    """
    load_modules(all_modules, config_dir, patch_dir)


def load_module(module, config_dir, patch_dir=None):
    """
    """
    return load_modules({module}, config_dir, patch_dir)
