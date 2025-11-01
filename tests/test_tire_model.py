
import pytest
from simple_tire_model.tire_model import simple_tire_model

def test_zero_inputs():
    """Test that the model returns zero forces and moments for zero inputs."""
    result = simple_tire_model(0, 0, 0)
    assert result["lateral_force"] == 0
    assert result["longitudinal_force"] == 0
    assert result["aligning_moment"] == 0

def test_pure_slip_angle():
    """Test the model with a pure slip angle."""
    result = simple_tire_model(0.1, 1000, 0)
    assert result["lateral_force"] != 0
    assert result["longitudinal_force"] == 0
    assert result["aligning_moment"] != 0

def test_pure_slip_ratio():
    """Test the model with a pure slip ratio."""
    result = simple_tire_model(0, 1000, 0.1)
    assert result["lateral_force"] == 0
    assert result["longitudinal_force"] != 0
    assert result["aligning_moment"] == 0
