"""Defines the standard plugin interface and command line plugin interface.

Plugins are a simple way for the end user to tack on extra functionality to a
simulation. Simply inherit from either of the classes defined here and start
overriding the default callback functions. Don't forget to add the pluging to
the simulation runner!
"""


class Plugin(object):
    """Defines the basic interface all simulation plugins should adhere to.
    """
    def __init__(self):
        super(Plugin, self).__init__()

    def arguments(self, parser):
        """Called by the simulation runner to add the plugin's command line
        arguments to the parser.

        The logging system is not enabled during this step.
        """
        pass

    def initialize(self, sim, args):
        """Plugin function called by the simulation before taking the first
        step.

        This could be used to change initial conditions or something similar.
        """
        pass

    def prestep(self, sim):
        """Plugin function called by the simulation prior to every step.
        """
        pass

    def poststep(self, sim):
        """Plugin function called by the simulation right after every step.
        """
        pass

    def cleanup(self, sim):
        """Plugin function called by the simulation just before exiting.
        """
        pass
