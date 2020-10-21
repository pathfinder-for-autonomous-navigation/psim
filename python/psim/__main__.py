
import lin
from . import (
  SimulationRunner,
  plugins,
)

SimulationRunner([
  plugins.StopOnSteps(),
  plugins.Plotter()
]).run()
