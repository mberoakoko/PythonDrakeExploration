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
    diagram = builder.Build()
    diagram.set_name("diagram")
    #
    plot_system_graphviz(diagram)
    plt.show()



if __name__ == "__main__":
    simple_diagram_builder()

