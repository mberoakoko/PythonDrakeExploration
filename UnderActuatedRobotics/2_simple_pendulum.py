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
from pydrake.systems.framework import BasicVector, System, AbstractValues
from pydrake.examples import PendulumGeometry, PendulumParams, PendulumPlant
import matplotlib
from matplotlib.axes import Axes
from matplotlib.figure import Figure
import copy
matplotlib.use("TkAgg")

plt.rcParams.update({'font.size': 8})
plt.style.use("bmh")


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
    pendulum: System | PendulumPlant = builder.AddSystem(PendulumPlant())
    saturation: System | Saturation = builder.AddSystem(Saturation(min_value=[-3], max_value=[3] ))
    builder.Connect(saturation.get_output_port(0), pendulum.get_input_port(0))
    controller: System | EnergyShapingController = builder.AddSystem(EnergyShapingController(pendulum))
    builder.Connect(pendulum.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), saturation.get_input_port(0))

    logger = builder.AddSystem(VectorLogSink(2))
    builder.Connect(pendulum.get_output_port(0), logger.get_input_port(0))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    figure: Figure = plt.figure(figsize=(20, 9))
    ax_1: Axes = figure.add_subplot(131)
    ax_2: Axes = figure.add_subplot(132)
    ax_3: Axes = figure.add_subplot(133)
    ax_1.set_xlabel("theta")
    ax_1.set_ylabel("theta_dot")
    ax_1.set_title("Energy Shaping Control")
    for i in range(5):
        context.SetTime(0.0)
        context.SetContinuousState(
            np.random.randn(
                2,
            )
        )
        simulator.Initialize()
        simulator.AdvanceTo(10)
        log = logger.FindLog(context)
        ax_1.plot(log.data()[0, :], log.data()[1, :])
        ax_2.plot(log.data()[0, :])
        ax_3.plot(log.data()[1, :])
        log.Clear()

    plt.tight_layout()
    plt.show()

def balancing_lqr_factory(pendulum: PendulumPlant = None):
    context = pendulum.CreateDefaultContext()
    pendulum.get_input_port(0).FixValue(context, [0])
    context.SetContinuousState([np.pi, 0])

    Q = np.diag([10.0, 1])
    R = np.diag([1])

    linearized_pendulum = Linearize(pendulum, context)
    (K, S) = LinearQuadraticRegulator(
        linearized_pendulum.A(), linearized_pendulum.B(), Q, R
    )
    return K, S

class SwingUpAndBalanceController(LeafSystem):
    def __init__(self, pendulum: PendulumPlant):
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("state", 2)
        self.DeclareVectorOutputPort("control", 1, self.__output)
        (self.__k, self.__s) = balancing_lqr_factory(pendulum)
        self.__energy_shaping: EnergyShapingController = EnergyShapingController(pendulum)
        self.__ctx_energy_shaping = self.__energy_shaping.CreateDefaultContext()


    def __output(self, ctx: Context, output: BasicVector) -> None:
        pendulum_state = self.get_input_port(0).Eval(ctx)
        x_bar = copy.copy(pendulum_state)
        x_bar[0] = wrap_to(x_bar[0], 0, 2.0 * np.pi) - np.pi

        if x_bar.dot(self.__s.dot(x_bar)) <= 2:
            output.SetFromVector(-self.__k.dot(x_bar))

        else:
            self.__energy_shaping.get_input_port(0).FixValue(self.__ctx_energy_shaping, pendulum_state)
            output.SetFromVector(
                self.__energy_shaping.get_output_port(0).Eval(self.__ctx_energy_shaping)
            )



def swing_up_pendulum_control_demo(meshcat=StartMeshcat())-> None:
    builder = DiagramBuilder()
    pendulum: PendulumPlant | System = builder.AddSystem(PendulumPlant())
    saturation = builder.AddSystem(Saturation(min_value=np.array([-3]), max_value=np.array([3])))
    builder.Connect(saturation.get_output_port(0), pendulum.get_input_port(0))
    controller = builder.AddSystem(SwingUpAndBalanceController(pendulum))

    builder.Connect(pendulum.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), saturation.get_input_port(0))

    # Visualization
    pend_scene_graph: SceneGraph | System = builder.AddSystem(SceneGraph())
    PendulumGeometry.AddToBuilder(builder, pendulum.get_state_output_port(), scene_graph=pend_scene_graph)
    MeshcatVisualizer.AddToBuilder(builder, scene_graph=pend_scene_graph, meshcat=meshcat)
    logger: VectorLogSink | System = builder.AddSystem(VectorLogSink(2))
    builder.Connect(pendulum.get_output_port(0), logger.get_input_port(0))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()


    figure: Figure = plt.figure(figsize=(16, 9))
    ax_1: Axes = figure.add_subplot(131)
    ax_2: Axes = figure.add_subplot(132)
    ax_3: Axes = figure.add_subplot(133)
    ax_1.set_xlabel("theta")
    ax_1.set_ylabel("theta_dot")
    ax_1.set_title("Energy Shaping Control")
    for i in range(5):
        context.SetTime(0.0)
        context.SetContinuousState(
            np.random.randn(
                2,
            )
        )
        simulator.Initialize()
        simulator.AdvanceTo(10)
        log = logger.FindLog(context)
        ax_1.plot(log.data()[0, :], log.data()[1, :])
        ax_2.plot(log.data()[0, :])
        ax_3.plot(log.data()[1, :])
        log.Clear()

    plt.tight_layout()
    plt.show()




if __name__ == "__main__":
    swing_up_pendulum_control_demo()