import numpy as np
from pydrake import symbolic
from pydrake.systems import primitives
from pydrake.systems.framework import LeafSystem, Context, ContinuousState

import matplotlib.pyplot as plt
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput

x = symbolic.Variable("x")

def continous_dynamics()-> primitives.SymbolicVectorSystem:
    return primitives.SymbolicVectorSystem(
        state=np.array([x]),
        dynamics=np.array([-x + x**3]),
        output=np.array([x])
    )

def discrete_dynamics()-> primitives.SymbolicVectorSystem:
    return primitives.SymbolicVectorSystem(
        state=np.array([x]),
        dynamics=np.array([-x + x**3]),
        output=np.array([x]),
        time_period=0.01
    )

class SimpleContinousTimeSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        state_index = self.DeclareContinuousState(1)
        self.DeclareStateOutputPort("y", state_index)

    def DoCalcTimeDerivatives(self, context: Context, derivatives: ContinuousState):
        x =  context.get_continuous_state_vector().GetAtIndex(0)
        x_dot = -10*x + x**2
        derivatives.get_mutable_vector().SetAtIndex(0, x_dot)


class SimpleDiscreteTimeSystem(LeafSystem):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        state_index = self.DeclareDiscreteState(1)
        self.DeclareStateOutputPort("y", state_index)
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=0.1,
            offset_sec=0.0,
            update=self.Update
        )

    def Update(self, context: Context, discrete_state):
        x = context.get_discrete_state_vector().GetAtIndex(0)
        x_next = x**3
        discrete_state.get_mutable_vector().SetAtIndex(0, x_next)


def simulate_continous_system():
    builder = DiagramBuilder()
    system = builder.AddSystem(SimpleContinousTimeSystem())
    logger = LogVectorOutput(system.get_output_port(0), builder)
    diagram = builder.Build()

    context = diagram.CreateDefaultContext()
    context.SetContinuousState(np.array([0.9], dtype=np.float64))

    sim = Simulator(diagram, context)
    sim.AdvanceTo(10)

    log = logger.FindLog(context)
    plt.figure()
    plt.plot(log.sample_times(), log.data().transpose())
    plt.xlabel("Time (s)")
    plt.ylabel("Y")
    plt.show()


def simulate_discrete_system():
    builder = DiagramBuilder()
    system = builder.AddSystem(SimpleDiscreteTimeSystem())




if __name__ == "__main__":
    simulate_continous_system()