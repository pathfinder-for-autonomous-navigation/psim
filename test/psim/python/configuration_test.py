
import lin
import psim

import pytest

import sys


def test_configuration():
  """Test configuration parsing and data access from Python.

  This test should guarantee all data types supported by configuration files can
  also be read into Python.
  """
  config = psim.Configuration('test/psim/python/configuration_test_config.txt')

  to = config['to']
  assert type(to) is int
  assert to == 0

  tf = config['tf']
  assert type(tf) is int
  assert tf == 1000

  alpha = config['alpha']
  assert type(alpha) is float
  assert alpha == 0.1

  x = config['x']
  assert type(x) is lin.Vector2
  assert x[0] == 1.0 and x[1] == 0.0

  y = config['y']
  assert type(y) is lin.Vector2
  assert y[0] == 0.0 and y[1] == 1.0

  w = config['w']
  assert type(w) is lin.Vector3
  assert w[0] == 0.1 and w[1] == 2.0 and w[2] == 3.0

  q = config['q']
  assert type(q) is lin.Vector4
  assert q[0] == 0.0 and q[1] == 0.0 and q[2] == 0.0 and q[3] == 1.0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
