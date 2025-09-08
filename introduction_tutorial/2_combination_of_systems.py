import matplotlib.pyplot as plt
import matplotlib
import numpy as np

from pydrake.examples import PendulumPlant
from pydrake.systems.controllers import PidController
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput
from pydrake.systems.drawing import plot_system_graphviz

matplotlib.use('TkAgg')


def simple_diagram_builder():
    builder = DiagramBuilder()
    pendulum = builder.AddNamedSystem("PendulumPlant", PendulumPlant())
    controlller = builder.AddNamedSystem(
        "Controller",
        PidController(
            kp=np.array([1.0]),
            ki=np.array([1.0]),
            kd=np.array([1.0]),
        )
    )

    builder.Connect(pendulum.get_state_output_port(), controlller.get_input_port_estimated_state())
    builder.Connect(controlller.get_output_port_control(), pendulum.get_input_port())

    builder.ExportInput(controlller.get_input_port_desired_state())
    builder.ExportOutput(pendulum.get_state_output_port())

    logger = LogVectorOutput(pendulum.get_state_output_port(), builder)
    logger.set_name("vec_output_logger")
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
    simple_diagram_builder()

