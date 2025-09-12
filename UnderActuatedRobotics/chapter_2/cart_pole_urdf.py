import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    LinearQuadraticRegulator,
    MeshcatVisualizer,
    Parser,
    Simulator,
    StartMeshcat,
)

base_urdf = """
  <link name="base">

    <inertial>
      <origin xyz="0 0 0" />
      <mass value="1" />
    </inertial>

    <visual>
      <origin xyz="0 0 0" />
      <geometry>
        <box size=".5 .2 .2" />
      </geometry>
      <material>
        <color rgba="0 1 0 1" />
      </material>
    </visual>

    <visual>
      <origin xyz=".15 0 -.15" rpy="0 0 0" />
      <geometry>
        <sphere radius=".05" />
      </geometry>
      <material>
        <color rgba="0 0 0 1" />
      </material>
    </visual>

    <visual>
      <origin xyz="-.15 0 -.15" rpy="0 0 0" />
      <geometry>
        <sphere radius=".05" />
      </geometry>
      <material>
        <color rgba="0 0 0 1" />
      </material>
    </visual>
  </link>
"""

pendulum_link = """ 
  <link name="pendulum0">
  <inertial>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <mass value="1" />
        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0" />
  </inertial>
  
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
        <sphere radius="0.05" />
    </geometry>
    <material>
        <color rgba="0 1 0 1" />
      </material>
  </visual>

  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
        <sphere radius="0.05" />
    </geometry>
    <material>
        <color rgba="0 1 0 1" />
      </material>
  </visual>


  </link>
"""