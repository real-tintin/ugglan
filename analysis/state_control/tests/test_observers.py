import pytest

from observers import *


@pytest.mark.parametrize("n_samples", [0, 10])
@pytest.mark.parametrize("alpha", [0, -1, 1])
@pytest.mark.parametrize("c", [0, 0.1, 1])
def test_reduced_observer(n_samples, alpha, c):
    reduced_observer(x_v=np.random.rand(1, n_samples),
                     u=np.random.rand(1, n_samples),
                     c=c, alpha=alpha)
