"""Python wrappers and utilities for PSim simulations.
"""

from _psim import (
    Configuration as _Configuration,
    # Import C++ simulations
    SingleOrbitGnc,
)

# ASCII art from:
#   http://patorjk.com/software/taag/#p=display&f=3D Diagonal&t=PSIM
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


class Simulation(object):
    """Small wrapper for a PSim simulation.

    This is the main entrypoint for end users and allows them to override the
    step function and add in there own actions accordingly.
    """
    def __init__(self, sim, config, quiet = False):
        super(Simulation, self).__init__()

        # Generate the simulation
        self._simulation = sim(_Configuration(config))

        if not quiet:
          print(_BANNER)

    def __getitem__(self, name):
        """Retrives a state field from the underlying simulation.
        """
        return self._simulation[name]

    def __setitem__(self, name, value):
        """Sets a state field in the underlying simulation.
        """
        self._simulation[name] = value

    def step(self):
        """Steps the underlying simulation forward in time.
        """
        self._simulation.step()
