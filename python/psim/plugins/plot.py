"""Data driven plotting utility consisting of a pluging and small set of helper
classes.
"""

from psim.plugins import Plugin
from psim.utilities import get_plotting_files

from matplotlib import pyplot as plt

import logging
import yaml

log = logging.getLogger(__name__)


def _mangle_field(array):
    """Mangle a field name from a data array name.
    """
    if array.endswith('.x') or array.endswith('.y') or array.endswith('.z') or \
            array.endswith('.w'):
        return array[:-2]
    else:
        return array


class Plot(object):
    """Represents a single plot managed by the plotter plugin.
    """
    def __init__(self, **kwargs):
        super(Plot, self).__init__()

        self.__dict__.update(kwargs)

        if self.__dict__.get('z', None):
            self.z = self.z if type(self.z) == list else [self.z]
        else:
            self.y = self.y if type(self.y) == list else [self.y]

        self._arrays = None
        self._fields = None

    @property
    def fields(self):
        """Retrieves a set of fields this plot needs to have logged in order to
        display it's plot upon simulation termination.
        """
        if not self._fields:
            self._fields = set()
            for array in self.arrays:
                self._fields.add(_mangle_field(array))

        return self._fields

    @property
    def arrays(self):
        """Retrieves a set of arrays this plot needs to have generated in order
        to display it's plot upon simulation termination.
        """
        if not self._arrays:
            self._arrays = {self.x}
            self._arrays.update({y for y in (self.y if type(self.y) == list else [self.y])})
            if self.__dict__.get('z', None):
                self._arrays.update({z for z in (self.z if type(self.z) == list else [self.z])})

        return self._arrays

    def plot(self, arrays):
        """Displays a plot.
        """
        if self.__dict__.get('z', None):
            log.debug('Plotting %s against (\'%s\', \'%s\')', str(self.z), self.x, self.y)

            f = plt.figure()
            ax = f.add_subplot(111, projection='3d')
            for _z in self.z:
                ax.plot(arrays[self.x], arrays[self.y], arrays[_z], label=_z)
            ax.legend()
            ax.set_xlabel(self.x)
            ax.set_ylabel(self.y)
            f.show()
        else:
            log.debug('Plotting %s against \'%s\'', str(self.y), self.x)

            f = plt.figure()
            ax = f.add_subplot(111)
            for _y in self.y:
                ax.plot(arrays[self.x], arrays[_y], label=_y)
            ax.legend()
            ax.set_xlabel(self.x)
            f.show()


class Plotter(Plugin):
    """Logs telemetry to display in a set of plots upon simulation termination.
    """
    def __init__(self, plots=None, step=1):
        super(Plotter, self).__init__()

        self._plots = plots if not plots or type(plots) == list else [plots]
        self._step = step
        self._n = 0

    def arguments(self, parser):
        super(Plotter, self).arguments(parser)

        _plots_default = self._plots if not self._plots else ','.join(self._plots)
        parser.add_argument(
            '-p', '--plots', type = str, default = _plots_default,
            help = 'comma separated list of plotting files used to log and ' +
            'plot data over the course of the simulation'
        )
        parser.add_argument(
            '-ps', '--plots-step', type = int, default = self._step,
            help = 'step interval at which data is recorded for plotting'
        )

    def initialize(self, sim, args):
        """Parses the plotting configuration files and determines what state
        fields must be logged during the simulation.
        """
        super(Plotter, self).initialize(sim, args)

        def _mangle_fields(plot):
            """Mangle required fields from a dictionary representing a plot.
            """
            _arrays = plot.get('x', []) + plot.get('y', []) + plot.get('z', [])
            return {_mangle_field(_array) for _array in _arrays}

        def _stream_plots(plots):
            """Generates plot dictionaries from a list of plots files.
            """
            for plot in plots:
                with open(plot, 'r') as istream:
                    _plots = yaml.safe_load(istream)
                    for _plot in _plots if type(_plots) == list else [_plots]:
                        yield Plot(**_plot)

        if not args.plots:
            log.warning('No plots specified via the command line; defaulting to %s.', str(self._plots))
        else:
            self._plots = args.plots.split(',')
            log.info('Overriding plots via the command line to %s', self._plots)

        log.debug('Loading plots from the following configuration files: %s', get_plotting_files(self._plots))

        fields = dict()
        plots = list()
        for plot in _stream_plots(get_plotting_files(self._plots)):
            for field in plot.fields:
                if not fields.get(field, None):
                    fields[field] = list()

            plots.append(plot)

        self._fields = fields
        self._plots = plots

    def poststep(self, sim):
        """Periodically logs the necessary fields for plotting upon termination
        of the simulation.
        """
        super(Plotter, self).poststep(sim)

        self._n = self._n + 1
        if self._step > 0 and self._n % self._step == 0:
            self._n = 0
            for k, v in self._fields.items():
                v.append(sim[k])

    def cleanup(self, sim):
        super(Plotter, self).cleanup(sim)

        # Ignore plots if the step was set to zero
        if not self._step > 0:
            log.warning('Skipping plotting because the plot step wasn\'t greater than zero')
            return

        log.info('Generating plots...')

        # Generate direct data arrays for plotting
        arrays = dict()
        for _arrays in [plot.arrays for plot in self._plots]:
            for _array in _arrays:
                if not arrays.get(_array, None):
                    if _array.endswith('.x'):
                        arrays[_array] = [x[0] for x in self._fields[_mangle_field(_array)]]
                    elif _array.endswith('.y'):
                        arrays[_array] = [x[1] for x in self._fields[_mangle_field(_array)]]
                    elif _array.endswith('.z'):
                        arrays[_array] = [x[2] for x in self._fields[_mangle_field(_array)]]
                    elif _array.endswith('.w'):
                        arrays[_array] = [x[3] for x in self._fields[_mangle_field(_array)]]
                    else:
                        arrays[_array] = [x for x in self._fields[_mangle_field(_array)]]

        # Dump the logged fields
        del self._fields

        # Loop through plots
        for plot in self._plots:
            plot.plot(arrays)

        # Block on user input
        log.info('Plotting complete! Press [ENTER] to continue.')
        input()
