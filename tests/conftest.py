"""pytest fixtures — shared setup for all bin_picking tests"""
import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'vision'))


@pytest.fixture
def cam_params():
    return {'cx': 0.47, 'cy': 0.0, 'cz': 0.5}


@pytest.fixture
def bin_params():
    return {
        'bcx': 0.5, 'bcy': 0.0, 'bcz': 0.05,
        'bsx': 0.4, 'bsy': 0.3, 'bsz': 0.15,
    }
