import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from pydrake.all import (
    DiagramBuilder,
    DiscreteAlgebraicRiccatiEquation,
    LeafSystem,
    LinearSystem,
    MeshcatVisualizer,
    MultilayerPerceptron,
    PerceptronActivationType,
    RandomGenerator,
    Rgba,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
    StartMeshcat,
    ZeroOrderHold,
)
from pydrake.examples import AcrobotPlant, PendulumGeometry, PendulumPlant


