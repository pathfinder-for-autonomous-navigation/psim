"""Random helper functions used throughout the rest of PSim.
"""

from psim import sims

import logging
import os

log = logging.getLogger(__name__)


def get_simulation_type(simulation):
    """Translates a string name to a simulation type.
    """
    types = list()
    for t in dir(sims):
        if t != 'Configuration' and not t.startswith('_'):
            types.append(t)

    if simulation not in types:
        raise RuntimeError(
            'Invalid simulation type requested. You can only request one of ' +
            'the following: ' + str(types)
        )

    return getattr(sims, simulation)


def _get_files(prefix, suffix, files):
    _files = list()
    for file in files:
        file = prefix + file + suffix
        if not os.path.isfile(file):
            log.error('Cannot find the following requested file: "%s"', file)
        else:
            _files.append(file)
    
    return _files


def get_configuration_files(configs):
    _configs = _get_files('config/parameters/', ".txt", configs)
    if len(configs) > len(_configs):
        log.warning('Only %d of the %d requested configuration files were found.', len(_configs), len(configs))

    return _configs


def get_plotting_files(plots):
    _plots = _get_files('config/plots/', ".yml", plots)
    if len(plots) > len(_plots):
        log.warning('Only %d of the %d requested plot files were found.', len(_plots), len(plots))

    return _plots
