
import lin
from . import (
  SimulationRunner,
  plugins,
)

SimulationRunner([
  plugins.Plotter(),
  plugins.Snapshot(),
  plugins.StopOnSteps(),
]).run()
