import matplotlib.pyplot as plt
import mpld3
import numpy as np
from pydrake.all import (
    DiagramBuilder,
    LeafSystem,
    Linearize,
    LinearQuadraticRegulator,
    MeshcatVisualizer,
    Saturation,
    SceneGraph,
    Simulator,
    StartMeshcat,
    VectorLogSink,
    wrap_to,
    Context
)
from pydrake.systems.framework import BasicVector
from pydrake.examples import PendulumGeometry, PendulumParams, PendulumPlant


class EnergyShapingController(LeafSystem):
    def __init__(self, pendulum: PendulumPlant):
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("state", 2)
        self.DeclareVectorOutputPort("control", 1, self.perform_output_calculation)
        self.__pendulum: PendulumPlant = pendulum
        self.__pendulum_context = self.__pendulum.CreateDefaultContext()
        self.__desired_energy: float = 0
        self.__set_pendulum_params(PendulumParams())


    def perform_output_calculation(self, ctx: Context, output: BasicVector) -> None:
        pendulum_state = self.get_input_port(0).Eval(ctx)
        self.__pendulum_context.SetContinuousState(pendulum_state)
        params = self.__pendulum_context.get_numeric_parameter(0)
        theta_dot = pendulum_state[1]
        total_energy = self.__pendulum.EvalPotentialEnergy(self.__pendulum_context) + self.__pendulum.EvalKineticEnergy(self.__pendulum_context)
        output.SetAtIndex(0, params.damping() * theta_dot - 0.1 * theta_dot * (total_energy - self.__desired_energy) )


    def __set_pendulum_params(self, pendulum_params: PendulumParams):
        (self.__pendulum_context
            .get_mutable_numeric_parameter(0)
            .SetFromVector(
                        pendulum_params.CopyToVector()
            )
        )
        self.__pendulum_context.SetContinuousState([np.pi, 0])
        self.__desired_energy = self.__pendulum.EvalPotentialEnergy(self.__pendulum_context)


def energy_shaping_demo():
    builder = DiagramBuilder()
    pendulum: PendulumPlant = builder.AddSystem(PendulumPlant())
    saturation: Saturation = builder.AddSystem(Saturation(min_value=[-3], max_value=[3] ))
    builder.Connect(saturation.get_output_port(0), pendulum.get_input_port(0))
    controller: EnergyShapingController = builder.AddSystem(EnergyShapingController(pendulum))
    builder.Connect(pendulum.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), saturation.get_input_port(0))

    logger = builder.AddSystem(VectorLogSink(2))
    builder.Connect(pendulum.get_output_port(0), saturation.get_input_port(0))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()



if __name__ == "__main__":
    meshcat = StartMeshcat()
    count = 0
    while True:
        count += 1
