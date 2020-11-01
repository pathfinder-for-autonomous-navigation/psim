"""Set of plugins used to generate a 'snapshot' of the simulation state.
"""

from psim.plugins import Plugin

import logging

log = logging.getLogger(__name__)


class Snapshot(Plugin):
    """Captures a snapshot of the simulation state upon termination and saves it
    to the specified file.
    """
    def __init__(self, snapshot=None):
        super(Snapshot, self).__init__()

        self._snapshot = snapshot

    def arguments(self, parser):
        super(Snapshot, self).arguments(parser)

        parser.add_argument(
            '--snapshot', type=str, help='specifies the output file for the ' +
            'simulation snapshot upon simulation termination.'
        )

    def initialize(self, sim, args):
        super(Snapshot, self).initialize(sim, args)

        if not args.snapshot:
            if not self._snapshot:
                log.warning('No snapshot file specified; snapshot feature disabled.')
            else:
                log.warning('No snapshot file specified; default to "%s"', self._snapshot)

            return
        
        self._snapshot = args.snapshot
        log.info('Saving simulation snapshot to "%s" upon simulation termination.', self._snapshot)

    def cleanup(self, sim):
        super(Snapshot, self).cleanup(sim)

        if not self._snapshot:
            return

        _fields = [
            'truth.t.ns',
            'truth.leader.orbit.r',
            'truth.leader.orbit.r',
            'truth.leader.attitude.q.body_eci',
            'truth.leader.attitude.w',
            'truth.leader.wheels.w',
            'truth.follower.orbit.r',
            'truth.follower.orbit.r',
            'truth.follower.attitude.q.body_eci',
            'truth.follower.attitude.w',
            'truth.follower.wheels.w',
        ]
        
        log.info('Saving simulation snapshot to "%s"', self._snapshot)
        with open(self._snapshot, 'w') as ostream:
            for _field  in _fields:
                print(_field, sim.get(_field), file=ostream)
