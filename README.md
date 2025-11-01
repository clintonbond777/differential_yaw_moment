
# Simple Tire Model

A simplified tire model to calculate tire forces and moments based on slip angle, vertical load, and slip ratio.

## Installation

To install the package, run:

```bash
pip install .
```

## Usage

```python
from simple_tire_model.tire_model import simple_tire_model

slip_angle = 0.1  # radians
vertical_load = 1000  # Newtons
slip_ratio = 0.05

forces_moments = simple_tire_model(slip_angle, vertical_load, slip_ratio)

print(forces_moments)
```
