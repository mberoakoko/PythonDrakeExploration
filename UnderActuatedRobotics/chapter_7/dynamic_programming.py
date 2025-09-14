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
from numpy.typing import NDArray
import dataclasses

@dataclasses.dataclass(frozen=True)
class DoubleIntegrator:
    A: np.ndarray = dataclasses.field(default_factory=lambda : np.array([[0.0, 1.0], [0.0, 0.0]]))
    B: np.ndarray = dataclasses.field(default_factory=lambda : np.array([[0.0], [1.0]]))
    Q: np.ndarray = dataclasses.field(default_factory=lambda :0.1 * np.eye(2))
    R: np.ndarray = dataclasses.field(default_factory=lambda : np.eye(1))


@dataclasses.dataclass
class NeuralFittedValueIteration:
    time_step: float
    discount_factor: float
    plant: DoubleIntegrator = dataclasses.field(default_factory=lambda : DoubleIntegrator())

    def quadratic_regulator_cost(self, x: NDArray[float], u: NDArray[float]) -> float:
        return x.T @ self.plant.Q @ self.x  + u.T @ self.plant.R @ u

    @staticmethod
    def min_time_cost(x: NDArray[float], u: NDArray[float]) -> float:
        return 1 - np.isclose(x, np.zeros((2, 0))).all(axis=0)

    def min_time_solution(self, x: NDArray[float]) -> NDArray[float]:
        q: NDArray[float] = x[0, :]
        q_dot: NDArray[float] = x[1, :]
