import numpy as np
from pydrake.common.containers import namedview
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, LeafSystem, Context
from pydrake.trajectories import PiecewisePolynomial
from enum import StrEnum

class SwissArmyKnife(LeafSystem):
    class InputPorts(StrEnum):
        PORT_A= "port_a"
        PORT_B= "port_b"

    class OutputPorts(StrEnum):
        SUM_PORT = "sum"
        PRODUCT_PORT = "product"
        INNER_PRODUCT_PORT = "inner_product"

    def __init__(self):
        LeafSystem.__init__(self)
        self.__a_port = self.DeclareVectorInputPort(name=SwissArmyKnife.InputPorts.PORT_A, size=2)
        self.__b_port = self.DeclareVectorInputPort(name="port_b", size=2)

        self.DeclareVectorOutputPort(
            name=SwissArmyKnife.OutputPorts.SUM_PORT,
            size=2,
            calc=self.__calc_sum
        )

        self.DeclareVectorOutputPort(
            name=SwissArmyKnife.OutputPorts.PRODUCT_PORT,
            size=2,
            calc=self.__calc_difference
        )
        #
        self.DeclareVectorOutputPort(
            name=SwissArmyKnife.OutputPorts.INNER_PRODUCT_PORT,
            size=1,
            calc=self.__calc_inner_product
        )

    def __calc_sum(self, context: Context, output_val: BasicVector):
        a = self.__a_port.Eval(context)
        b = self.__b_port.Eval(context)
        output_val.SetFromVector( a + b)

    def __calc_difference(self, context: Context, output: BasicVector):
        a = self.__a_port.Eval(context)
        b = self.__b_port.Eval(context)
        output.SetFromVector(a - b)

    def __calc_inner_product(self, context: Context, output: BasicVector):
        a = self.__a_port.Eval(context)
        b = self.__b_port.Eval(context)
        output.SetFromVector([np.inner(a, b)])

def run_test_on_swiss_army_knife():
    swiss_army_knife = SwissArmyKnife()
    context = swiss_army_knife.CreateDefaultContext()
    swiss_army_knife.GetInputPort(SwissArmyKnife.InputPorts.PORT_A).FixValue(context, [3, 4])
    swiss_army_knife.GetInputPort(SwissArmyKnife.InputPorts.PORT_B).FixValue(context, [3, 45])
    print(f"sum : {swiss_army_knife.GetOutputPort(SwissArmyKnife.OutputPorts.SUM_PORT).Eval(context)}")
    print(f"product : {swiss_army_knife.GetOutputPort(SwissArmyKnife.OutputPorts.PRODUCT_PORT).Eval(context)}")
    print(f"InnerProduct : {swiss_army_knife.GetOutputPort(SwissArmyKnife.OutputPorts.INNER_PRODUCT_PORT).Eval(context)}")

if __name__ == "__main__":
    run_test_on_swiss_army_knife()
