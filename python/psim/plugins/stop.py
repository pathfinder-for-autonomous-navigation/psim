"""Set of plugins acting as stop conditions for a simulation.
"""

from psim.plugins import Plugin

import logging

log = logging.getLogger(__name__)


class StopOnSteps(Plugin):
    """Acts as a stopping condition on the step count of a simulation.
    """
    def __init__(self, n=0):
        super(StopOnSteps, self).__init__()

        self._n = n
        self._steps = 0
        self._percent = 10

    def arguments(self, parser):
        super(StopOnSteps, self).arguments(parser)

        parser.add_argument(
            '-s', '--steps', type = int, default = self._n, help = 'set the ' +
            'number of steps the simulation will stop on (a value less than ' +
            'or equal to zero prevents the simulation from stopping on its ' +
            'step count)'
        )

    def initialize(self, sim, args):
        super(StopOnSteps, self).initialize(sim, args)

        if not args.steps:
            log.warning('No maximum step count specified via the command line; defaulting to %d steps.', self._n)
        else:
            log.info('Overriding maximum step count via the command line to %d.', args.steps)
            self._n = args.steps

    def poststep(self, sim):
        super(StopOnSteps, self).poststep(sim)

        self._steps = self._steps + 1
        if self._steps / self._n >= float(self._percent) / 100.0:
            log.info('%d%% of the maximum allowable steps taken.', self._percent)
            self._percent = self._percent + 10
        if self._n <= 0 or self._steps < self._n:
            return

        log.info('Maximum step count of %d steps reached; halting the simulation.', self._n)
        sim.should_stop()
