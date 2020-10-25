
import lin
from psim import (
  SimulationRunner,
  plugins,
)

SimulationRunner([
  plugins.StopOnSteps(),
  plugins.Plotter()
]).run()
