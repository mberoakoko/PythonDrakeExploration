
import numpy as np
from pydrake.geometry import Meshcat
from pydrake.all import (
    ControllabilityMatrix,
    DiagramBuilder,
    Linearize,
    LinearQuadraticRegulator,
    MeshcatVisualizer,
    Saturation,
    SceneGraph,
    Simulator,
    StartMeshcat,
    WrapToSystem,
    OutputPort,
    System,
    LeafSystem,
    Sphere
)
from pydrake.geometry import Rgba
from pydrake.examples import AcrobotGeometry, AcrobotInput, AcrobotPlant, AcrobotState

import typing
from functools import partial

class MeshcatSliders(LeafSystem):
    """
    A system that outputs the values from meshcat sliders.

    An output port is created for each element in the list `slider_names`.
    Each element of `slider_names` must itself be an iterable collection
    (list, tuple, set, ...) of strings, with the names of sliders that have
    *already* been added to Meshcat via Meshcat.AddSlider().

    The same slider may be used in multiple ports.
    """

    def __init__(self, meshcat: Meshcat, slider_names: typing.List[str]):
        LeafSystem.__init__(self)

        self._meshcat = meshcat
        self._sliders = slider_names
        for i, slider_iterable in enumerate(self._sliders):
            port: OutputPort = self.DeclareVectorOutputPort(
                f"slider_group_{i}",
                len(slider_iterable),
                partial(self._DoCalcOutput, port_index=i),
            )
            port.disable_caching_by_default()

    def _DoCalcOutput(self, context, output, port_index):
        for i, slider in enumerate(self._sliders[port_index]):
            output[i] = self._meshcat.GetSliderValue(slider)


def acrobot_demo():
    builder = DiagramBuilder()
    acrobot = builder.AddSystem(AcrobotPlant())

    # Set up visualization
    scene_graph: SceneGraph | System = builder.AddSystem(SceneGraph())
    AcrobotGeometry.AddToBuilder(builder, acrobot.get_output_port(), scene_graph)
    meshcat.Delete()
    meshcat.Set2dRenderMode(xmin=-4, xmax=4, ymin=-4, ymax=4)
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    meshcat.AddSlider(
        "u",
        min=-5,
        max=5,
        step=0.1,
        value=0.0,
        decrement_keycode="ArrowLeft",
        increment_keycode="ArrowRight",
    )

    torque_system = builder.AddSystem(MeshcatSliders(meshcat, ["u"]))
    builder.Connect(torque_system.get_output_port(), acrobot.get_input_port())

    diagram = builder.Build()
    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # Set the initial conditions (theta1, theta2, theta1dot, theta2dot)
    context.SetContinuousState([1, 0, 0, 0])
    simulator.set_target_realtime_rate(1.0)
    print("Use the slider in the MeshCat controls to apply elbow torque.")
    print("Press 'Stop Simulation' in MeshCat to continue.")
    meshcat.AddButton("Stop Simulation")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)


if __name__ == "__main__":
    meshcat = StartMeshcat()
    acrobot_demo()

    # sphere = Sphere(1)
    #
    # # Add the sphere to the meshcat scene
    # meshcat.SetObject("my_sphere", sphere, rgba=Rgba(0.1, 0.1, 0.1, 1))

    input("Press Enter to stop the MeshCat server...")