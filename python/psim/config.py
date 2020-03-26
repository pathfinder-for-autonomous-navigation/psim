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
    return os.path.abspath(os.path.expanduser(path))


def _get_config_directories(config_dir, patch_dir, base_directory):
    # Ensure the base directory exists
    base_directory = _get_full_path(base_directory)
    if not os.path.isdir(base_directory):
        raise RuntimeError('Invalid base configuration directory: %s' % base_directory)

    # Ensure the specified directory is valid
    config_dir = _get_full_path(os.path.join(base_directory, config_dir))
    if not os.path.isdir(config_dir) or not config_dir.startswith(base_directory):
        raise RuntimeError('Invalid configuration directory: %s' % config_dir)

    # Build up a list of the standard configuration directories
    directories = list()
    while config_dir.startswith(base_directory):
        directories.append(config_dir)
        config_dir = _get_full_path(os.path.dirname(config_dir))
    directories.reverse()

    # Include the patch directory if provided
    if patch_dir:
        patch_dir = _get_full_path(patch_dir)
        if not os.path.isdir(patch_dir):
            raise RuntimeError('Invalid patch directory: %s' % patch_dir)
        directories.append(patch_dir)

    return directories


def load_modules(modules, config_dir, patch_dir=None, base_directory=_base_directory):
    """Load configuration data from the specified modules.

    Args:
        modules: Set of string naming modules to be loaded. Modules not
            recognized will be ignored.
        config_dir: Configuration directory (a relative path in "config/").
        patch_dir: Extra configuration directory taking precendance over all
            directories recursively discovered via "config_dir".
        base_directory: Root directory of the standard configuration files.

    Returns:
        Dictionary containing all of the configuration data. Each recognized
        module is a key in the dictionary.
    
    Throws:
        Runtime errors if directories are invalid.
    """
    def search_for_module_dictionary(directory, module):
        potential_json = _get_full_path(os.path.join(directory, module + ".json"))
        if os.path.isfile(potential_json):
            with open(potential_json, 'r') as _json:
                return json.load(_json)

        return dict()

    def search_for_sub_dictionary(dictionary, sub_dictionary):
        if (sub_dictionary in dictionary.keys()):
            _sub_dictionary = dictionary[sub_dictionary]
            if not (type(_sub_dictionary) is dict):
                raise RuntimeError('"%s" entry is expected to be a dictionary' % sub_dictionary)
            return _sub_dictionary

        return dict()

    # Determine which modules we actually need to worry about
    _global_modules = global_modules.intersection(modules)
    _satellite_modules = satellite_modules.intersection(modules)
    _log.debug('Loading the following global modules: %s', _global_modules)
    _log.debug('Loading the following satellite modules: %s', _satellite_modules)

    # Initialize the configuration data to empty dictionaries
    config = dict()
    for module in _global_modules:
        config[module] = dict()
    for module in _satellite_modules:
        config[module] = { "leader": dict(), "follower": dict() }

    # Loop over configuration directories
    for directory in _get_config_directories(config_dir, patch_dir, base_directory):
        _log.debug('Processing configuration data in "%s"', directory)

        # Update the global modules
        for module in _global_modules:
            _dictionary = search_for_module_dictionary(directory, module)
            config[module].update(_dictionary)

        # Update the satellite modules
        for module in _satellite_modules:
            # Look for shared, leader, and follower entries
            _dictionary = search_for_module_dictionary(directory, module)
            _shared = search_for_sub_dictionary(_dictionary, 'shared')
            _leader = search_for_sub_dictionary(_dictionary, 'leader')
            _follower = search_for_sub_dictionary(_dictionary, 'follower')

            # Prepare the new leader dictionary
            config[module]['leader'].update(_shared)
            config[module]['leader'].update(_leader)

            # Prepare the new follower dictionary
            config[module]['follower'].update(_shared)
            config[module]['follower'].update(_follower)

    return config


def load_module(module, config_dir, patch_dir=None, base_directory=_base_directory):
    """See the "load_modules" function
    """
    return load_modules({module}, config_dir, patch_dir, base_directory)


def load_all(config_dir, patch_dir=None, base_directory=_base_directory):
    """See the "load_modules" function
    """
    return load_modules(all_modules, config_dir, patch_dir, base_directory)
