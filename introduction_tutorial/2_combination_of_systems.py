import matplotlib.pyplot as plt
import matplotlib
from matplotlib.figure import Figure

import numpy as np

from pydrake.examples import PendulumPlant
from pydrake.systems.controllers import PidController
from pydrake.systems.framework import DiagramBuilder, Context, Diagram
from pydrake.systems.primitives import LogVectorOutput, VectorLogSink
from pydrake.systems.analysis import Simulator
from pydrake.systems.drawing import plot_system_graphviz
from pyparsing import lineEnd

# plt.rcParams.update({"figure.dpi": 300})
matplotlib.use('TkAgg')
plt.style.use("bmh")


def simple_diagram_builder():
    builder = DiagramBuilder()
    pendulum__ = builder.AddNamedSystem("PendulumPlant", PendulumPlant())
    controlller_ = builder.AddNamedSystem(
        "Controller",
        PidController(
            kp=np.array([100.0]),
            ki=np.array([100.0]),
            kd=np.array([10.0]),
        )
    )

    builder.Connect(pendulum__.get_state_output_port(), controlller_.get_input_port_estimated_state())
    builder.Connect(controlller_.get_output_port_control(), pendulum__.get_input_port())

    builder.ExportInput(controlller_.get_input_port_desired_state())
    builder.ExportOutput(pendulum__.get_state_output_port())

    logger__ = LogVectorOutput(pendulum__.get_state_output_port(), builder)
    logger__.set_name("vec_output_logger")
    #
    diagram_ = builder.Build()
    diagram_.set_name("diagram")

    # fig: Figure = plt.figure(figsize=(16, 9))
    return diagram_, pendulum__, controlller_, logger__

def simulate_system(diagram_: Diagram,pendulum_: PendulumPlant, controller: PidController, logger_: VectorLogSink) -> None:
    simulator = Simulator(diagram_)
    context: Context = simulator.get_mutable_context()
    desired_angle = np.pi / 2
    pendulum_context_ = diagram_.GetMutableSubsystemContext(pendulum_, context)
    print(f"\033[94m{pendulum_context_}\033[0m")
    (
        pendulum_context_
        .get_mutable_continuous_state_vector()
        .SetFromVector(
            np.array([desired_angle + 0.1, 0.2])
        )
    )

    (
        diagram_
        .get_input_port(0)
        .FixValue(context, [desired_angle, 0])
    )

    simulator.AdvanceTo(30)

    log = logger_.FindLog(simulator.get_context())
    t = log.sample_times()
    plt.figure(figsize=(16, 9))
    # Plot theta.
    plt.plot(t, log.data()[0, :], linewidth=0.7)
    # Draw a line for the desired angle.
    plt.plot([t[0], t[-1]], [desired_angle, desired_angle], 'g')
    plt.xlabel('time (seconds)')
    plt.ylabel('theta (rad)')
    plt.title('PID Control of the Pendulum')
    plt.tight_layout()
    plt.show()





if __name__ == "__main__":
    diagram_, pendulum, controller_, logger_ = simple_diagram_builder()
    simulate_system(diagram_, pendulum, controller_, logger_)

