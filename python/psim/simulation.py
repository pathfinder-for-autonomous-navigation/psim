"""Defines a set of classes that can be used to run a simulation.
"""

from . import utilities

from _psim import Configuration

import argparse
import logging

log = logging.getLogger(__name__)

_BANNER = """
,-.----.                                 ____   
\    /  \    .--.--.      ,---,        ,'  , `.
|   :    \  /  /    '. ,`--.' |     ,-+-,.' _ |
|   |  .\ :|  :  /`. / |   :  :  ,-+-. ;   , ||
.   :  |: |;  |  |--`  :   |  ' ,--.'|'   |  ;|
|   |   \ :|  :  ;_    |   :  ||   |  ,', |  ':
|   : .   / \  \    `. '   '  ;|   | /  | |  ||
;   | |`-'   `----.   \|   |  |'   | :  | :  |,
|   | ;      __ \  \  |'   :  ;;   . |  ; |--'
:   ' |     /  /`--'  /|   |  '|   : |  | ,
:   : :    '--'.     / '   :  ||   : '  |/
|   | :      `--'---'  ;   |.' ;   | |`-'
`---'.|                '---'   |   ;/ 
  `---`                        '---'
"""

_DESCRIPTION = """
Command line interface for running standalone PSim simulations.
"""


class Simulation(object):
    """Small wrapper around PSim simulations.

    This is the recommended entrypoint if you're not interested in using the
    plugin system and don't want to use the simulation runner.
    """
    def __init__(self, sim, configs):
        super(Simulation, self).__init__()

        self._sim = sim(Configuration(configs))

    def __getitem__(self, name):
        """Retrives a state field from the underlying simulation.
        """
        return self._sim[name]

    def __setitem__(self, name, value):
        """Sets a state field in the underlying simulation.
        """
        self._sim[name] = value

    def step(self):
        """Steps the underlying simulation forward in time.
        """
        self._sim.step()


class SimulationRunner(object):

    def __init__(self, plugins, args=None):
        super(SimulationRunner, self).__init__()

        self._plugins = plugins if type(plugins) == list else [plugins]
        self._should_stop = False

        # Build the argument parser
        parser = argparse.ArgumentParser(description=_DESCRIPTION)
        parser.add_argument(
            '-v', '--verbose', action = 'store_true', help = 'set the ' +
            'logging level to DEBUG instead of INFO'
        )

        for plugin in self._plugins:
            plugin.arguments(parser)

        parser.add_argument(
            '-c', '--configs', type = str, required = True, help = 'comma ' +
            'seprated list of configuration files used to initialize the ' +
            'simulation'
        )
        parser.add_argument(
            'simulation', metavar = 'SIM', type = str, help = 'sets the ' +
            'simulation type.'
        )

        # Parser command line arguments
        args = parser.parse_args(args)

        # Initialize the logger based on the verbosity flag(s)
        logging.basicConfig(
            format = '[%(asctime)s %(levelname)s] %(name)s: %(message)s',
            datefmt = '%I:%M:%S %p',
            level = logging.DEBUG if args.verbose else logging.INFO,
        )
        logging.getLogger('matplotlib').setLevel(logging.WARNING)

        print(_BANNER)
        log.info('Welcome to PSim!')
        log.info('Initializing the simulation...')

        # Determine the set of configuration files to load
        configs = utilities.get_configuration_files(args.configs.split(','))
        log.debug('Loading initial conditions from the following configuration files: %s', str(configs))

        # Determine the simulation type
        sim = utilities.get_simulation_type(args.simulation)
        log.debug('Simulation type set to "%s"', str(sim))

        # Construct the simulation
        self._sim = Simulation(sim, configs)

        # Initialize plugins
        for plugin in self._plugins:
            plugin.initialize(self, args)

        log.info('Initialization complete!')

    def __getitem__(self, name):
        """Retrives a state field from the underlying simulation.
        """
        return self._sim[name]

    def __setitem__(self, name, value):
        """Sets a state field in the underlying simulation.
        """
        self._sim[name] = value

    def should_stop(self):
        """Function available to plugins to allow them to signal the simulation
        should halt.
        """
        self._should_stop = True

    def run(self):
        """Runs the simulation and calls all of the appropriate callback
        functions.

        The simulation will continue running until a plugin signals the
        simulation should stop. Upon stopping, each plugin's cleanup callback
        will be called before returning.
        """
        log.info('Entering main loop...')

        while not self._should_stop:
            for plugin in self._plugins:
                plugin.prestep(self)
            
            self._sim.step()

            for plugin in self._plugins:
                plugin.poststep(self)

        log.info('Stop condition detected; exiting main loop and starting cleanup.')

        for plugin in self._plugins:
            plugin.cleanup(self)

        log.info('Simulation complete!')
